#!/usr/bin/env python3
from pymavlink import mavutil
import threading
import time
import json
import traceback
import queue


# ----------------------
# Low-level MAV IO layer
# ----------------------
class MavIO:
    """Centralized MAVLink IO.

    - real: underlying mavutil connection
    - read thread: reads messages and dispatches
    - send thread: serializes outgoing calls
    - waiters: temporary waiters for a specific message type
    - handlers: persistent handlers called for each message type
    """
    def __init__(self, real_mav, logger):
        self.real = real_mav  # real mavutil connection
        self.logger = logger
        self._running = True


        # dispatch structures
        self.handlers = {}  # mtype -> [callable(msg)]
        self.waiters = {}   # mtype -> [queue.Queue]
        self._handlers_lock = threading.Lock()

        # send queue for serialized writes: each item is (callable(real), done_event, result_container)
        self._send_q = queue.Queue()

        # start threads
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._read_thread.start()
        self._send_thread.start()

    # -----------------
    # Sending utilities
    # -----------------
    def _send_loop(self):
        while self._running:
            try:
                func, done_ev, container = self._send_q.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                # execute the user-provided func, passing the real mavutil object
                res = func(self.real)
                if container is not None:
                    container['result'] = res
                if done_ev is not None:
                    done_ev.set()
            except Exception as e:
                self.logger.append(f"MavIO: exception in send loop: {e}\n{traceback.format_exc()}")
                if container is not None:
                    container['exc'] = e
                if done_ev is not None:
                    done_ev.set()

    def send_sync(self, func, timeout=None):
        """Enqueue a callable(func(real_mav)) and block until it completes.
        Returns function result or raises exception captured inside.
        """
        done = threading.Event()
        container = {}
        self._send_q.put((func, done, container))
        completed = done.wait(timeout)
        if not completed:
            raise TimeoutError("MAV send timed out")
        if 'exc' in container:
            raise container['exc']
        return container.get('result', None)

    def send_async(self, func):
        """Enqueue a callable(func(real_mav)) without waiting."""
        self._send_q.put((func, None, None))

    # -----------------
    # Read / dispatch
    # -----------------
    def _read_loop(self):
        while self._running:
            try:
                msg = self.real.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                self.logger.append(f"MavIO: recv_match error: {e}")
                # pause then retry
                time.sleep(0.5)
                continue

            if not msg:
                continue

            mtype = msg.get_type()

            # 1) deliver to any persistent handlers
            with self._handlers_lock:
                handlers = list(self.handlers.get(mtype, []))
            for h in handlers:
                try:
                    h(msg)
                except Exception:
                    self.logger.append(f"MavIO: handler for {mtype} raised\n{traceback.format_exc()}")

            # 2) deliver to temporary waiters
            with self._handlers_lock:
                qs = self.waiters.get(mtype, [])
            for q in qs:
                try:
                    # non-blocking put, ignore if full
                    q.put_nowait(msg)
                except Exception:
                    pass

    # -----------------
    # Handler API
    # -----------------
    def register_handler(self, mtype, cb):
        with self._handlers_lock:
            self.handlers.setdefault(mtype, []).append(cb)

    def unregister_handler(self, mtype, cb):
        with self._handlers_lock:
            if mtype in self.handlers and cb in self.handlers[mtype]:
                self.handlers[mtype].remove(cb)

    def wait_for(self, mtype, timeout=None):
        """Block until a message of type mtype arrives (or timeout)."""
        q = queue.Queue(maxsize=1)
        with self._handlers_lock:
            self.waiters.setdefault(mtype, []).append(q)
        try:
            msg = q.get(timeout=timeout)
            return msg
        finally:
            # cleanup
            with self._handlers_lock:
                if mtype in self.waiters and q in self.waiters[mtype]:
                    self.waiters[mtype].remove(q)

    def stop(self):
        self._running = False
        # wake threads by putting no-op
        try:
            self._send_q.put((lambda m: None, None, None))
        except Exception:
            pass


# ----------------------
# MasterProxy & Sender
# ----------------------
class MavMavProxy:
    """A proxy that exposes the .mav.* send functions but routes them through MavIO's send queue."""
    def __init__(self, mavio: MavIO):
        self._mavio = mavio

    def __getattr__(self, name):
        # return a function that will execute real_mav.mav.<name>(*a,**k) inside send_sync
        def _call(*args, **kwargs):
            def _fn(real):
                # access real.mav.<name>
                fn = getattr(real.mav, name)
                return fn(*args, **kwargs)
            # use a reasonable timeout to avoid deadlocks in send
            return self._mavio.send_sync(_fn, timeout=10)
        return _call


class MavUtilProxy:
    """Proxy for top-level mavutil methods (motors_armed, set_mode, etc.)
    Calls to these are also routed through send_sync when they perform writes, or via
    direct attribute access for reads that don't touch serial (best-effort).
    """
    def __init__(self, mavio: MavIO):
        self._mavio = mavio

    def __getattr__(self, name):
        real = self._mavio.real
        attr = getattr(real, name, None)
        if callable(attr):
            # decide if it's a read or write. We'll conservatively route all callables
            # through send_sync to ensure serialized access (safe).
            def _call(*args, **kwargs):
                def _fn(real_in):
                    fn = getattr(real_in, name)
                    return fn(*args, **kwargs)
                return self._mavio.send_sync(_fn, timeout=10)
            return _call
        else:
            return attr


class MasterProxy:
    """Object passed to extra functions which acts like the proxy mavlink object

    - .mav -> MavMavProxy to execute low-level send functions
    - .recv_match(...) -> routed to MavIO.wait_for
    - any other attribute/method accessed is proxied to the underlying real mavutil
    """
    def __init__(self, mavio: MavIO):
        self._mavio = mavio
        # expose .mav and top-level methods
        self.mav = MavMavProxy(mavio)
        self._util = MavUtilProxy(mavio)

    def recv_match(self, type=None, blocking=True, timeout=None):
        # type is the message name (e.g., 'MISSION_REQUEST')
        # If blocking=True, wait_for will block; otherwise, try short timeout
        if not blocking and timeout is None:
            timeout = 0.0
        try:
            return self._mavio.wait_for(type, timeout=timeout)
        except queue.Empty:
            return None

    def __getattr__(self, name):
        # fallback to util proxy
        return getattr(self._util, name)


#MAVLink class that handles all the function
class MAVhandler:
    def __init__(self,port='/dev/ttyACM0',baud=57600, gps_rate = 5, local_rate = 5):
        self.uri = port
        self.boot_time = time.time()
        self.baud = baud
        self.gps_stream_rate = gps_rate
        self.local_rate = 10
        self.messages = [] 
        '''List of messages'''
        self.current_mode = None 
        '''read current mode'''
        self.state = {}
        '''GPS location'''
        self.home_gps = {} 
        self.home_set = False
        '''Home GPS'''
        try:
            real = mavutil.mavlink_connection(
                self.uri,
                baud=self.baud,
                autoreconnect=True
            )

            # -------------------------------------------------------
            # Wait specifically for the AUTOPILOT heartbeat
            # -------------------------------------------------------
            hb = None
            deadline = time.time() + 10.0

            while time.time() < deadline:
                msg = real.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)

                if not msg:
                    continue
                
                self.messages.extend([
                    f"HEARTBEAT from sys={msg.get_srcSystem()} ",
                    f"comp={msg.get_srcComponent()} ",
                    f"autopilot={msg.autopilot}"]
                )
                

                # Accept only Flight Controller heartbeat (ArduPilot)
                if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                    hb = msg
                    break

            if hb is None: #Falling back to default heartbeat
                self.messages.append("No autopilot heartbeat found. Falling back to default wait_heartbeat().")
                real.wait_heartbeat()
                self.system_id = real.target_system
                self.component_id = real.target_component

            else:
                # Force correct autopilot system/component
                real.target_system = hb.get_srcSystem()
                real.target_component = hb.get_srcComponent()

                self.system_id = real.target_system
                self.component_id = real.target_component

                self.messages.append(
                    f"Using AUTOPILOT target sys={self.system_id} comp={self.component_id}"
                )

            # Small stabilization delay
            time.sleep(0.1)

            # -------------------------------------------------------
            # Now create MavIO AFTER correct autopilot is selected
            # -------------------------------------------------------
            self.mavio = MavIO(real, self.messages)
            self.mav = MasterProxy(self.mavio)



            # request position stream
            try:
                def _req_fn(r):
                    r.mav.command_long_send(
                    self.system_id,
                    self.component_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    1000000/gps_rate,  # 10 Hz
                    0,0,0,0,0
                )
                self.mavio.send_sync(_req_fn, timeout=5)
            except Exception as e:
                self.messages.append(f"Failed to request position stream: {e}")

            # Request local position stream (NEW)
            try:
                def _req_local(r):
                    r.mav.command_long_send(
                    self.system_id,
                    self.component_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                    1000000/local_rate,   # 10 Hz
                    0,0,0,0,0
                )
                self.mavio.send_sync(_req_local, timeout=5)
            except Exception as e:
                self.messages.append(f"Failed to request local position stream: {e}")
            def _req_sys_status(r):
                r.mav.command_long_send(
                    self.system_id,
                    self.component_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                    1e6 / 5,
                    0, 0, 0, 0, 0
                )
            self.mavio.send_sync(_req_sys_status, timeout=5)

        except Exception as e:
            self.messages.append(f"Failed to initialize MAVLink: {e}\n{traceback.format_exc()}")
            raise
        self.mavio.register_handler('LOCAL_POSITION_NED', self._local_pos_handler)
        self.mavio.register_handler('GLOBAL_POSITION_INT', self._gps_handler)
        self.mavio.register_handler('HEARTBEAT', self._heartbeat_handler)
        self.mavio.register_handler('SYS_STATUS', self._battery_handler)


    def _battery_handler(self,msg):
        self.state["voltage"] = msg.voltage_battery
        self.state["current"] = msg.current_battery
        self.state["cpu"]     = msg.load

    def _local_pos_handler(self,msg):

        self.state['x'] = msg.x
        self.state['y'] = msg.y
        self.state['z'] = msg.z

        self.state['alt'] = -msg.z

        self.state['vx'] = msg.vx
        self.state['vy'] = msg.vy
        self.state['vz'] = msg.vz
# Register GPS handler
    def _gps_handler(self,msg):
        try:
            self.state["stamp"] = time.time() - self.boot_time
            self.state["latitude"] = msg.lat / 1e7
            self.state["longitude"] = msg.lon / 1e7
            self.state["altitude"] = msg.relative_alt / 1000.0
            # ? Set FC HOME only once, after valid GPS fix
            if not hasattr(self, "_fc_home_sent"):

                self.messages.append("? First GPS fix received -> Sending DO_SET_HOME now")

                self.set_home()

            # Store first GPS fix as startup HOME (only once)
            if not self.home_set:
                try:
                    self.home_gps = {
                        "lat": float(msg.lat) / 1e7,
                        "lon": float(msg.lon) / 1e7,
                        "alt": float(msg.alt) / 1000.0
                    }
                    self.home_set = True
                    self.messages.append(
                        f"Startup HOME stored: {self.home_gps['lat']:.7f}, {self.home_gps['lon']:.7f}, alt={self.home_gps['alt']:.2f}m"
                    )
                except Exception as e:
                    self.messages.append(f"Failed to store startup HOME: {e}")

        except Exception:
            self.messages.append(f"GPS handler failed: {traceback.format_exc()}")


    def _heartbeat_handler(self,msg):
        try:
            if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                return  # ignore GCS, companion, etc
            self.current_base_mode = msg.base_mode
            custom_mode = msg.custom_mode

            # Decode ArduPilot mode name
            try:
                mapping = self.mavio.real.mode_mapping()
                inv = {v: k for k, v in mapping.items()}
                mode_name = inv.get(custom_mode, f"UNKNOWN({custom_mode})")
            except Exception:
                mode_name = f"UNKNOWN({custom_mode})"

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            self.state = {
                "mode": mode_name,
                "armed": armed,
                "system_status": msg.system_status,
                "autopilot": msg.autopilot,
                "type": msg.type
            }

            self.current_mode = mode_name
        except Exception:
            self.messages.append(
                f"MAVLink HEARTBEAT handler error:\n{traceback.format_exc()}"
            )

    def set_home(self,current=True,lat=0,lon=0, alt=0):
        """set the home position at current location or given coordinates"""

        def _set_home_cmd(r):
            r.mav.command_int_send(
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                current,          # current (unused)
                0,          # autocontinue (unused)
                0.0, 0.0, 0.0,0.0,   # param2-4 unused (floats)
                lat,      # param5 latitude as int32
                lon,      # param6 longitude as int32
                alt       # param7 altitude (float, meters)
            )

        try:
            self.mavio.send_sync(_set_home_cmd, timeout=5)
            self.messages.append("? Flight Controller HOME successfully set")

        except Exception as e:
            self.messages.append(f"HOME set failed: {e}")

    def arm(self):
        self.mav.mav.command_long_send(
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

    def disarm(self):
        """Forced disarm"""
        try:
            mapping = self.mavio.real.mode_mapping()

            self.set_mode("STABILIZE")
            time.sleep(0.3)
            self.mav.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 21196, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self.messages.append(f"DISARM failed: {e}")


    def set_mode(self,mode): 
        """set ur modes as "STABILIZE/LOITER/GUIDED/AUTO/RTL/LAND"""
        try:
            mapping = self.mavio.real.mode_mapping()
        except Exception:
            mapping = None
        if mapping and mode in mapping:
            def _set_guided(r):
                r.set_mode(mapping[mode])
            self.mavio.send_sync(_set_guided, timeout=5)
        elif hasattr(self.mavio.real, 'set_mode_apm'):
            self.mavio.send_sync(lambda r: r.set_mode_apm(mode), timeout=5)
        else:
            self.messages.append("mapping/set_mode_apm unavailable; cannot force mode.")


    def set_servo(self, channel: int, pwm: int, sync: bool = True, timeout: float = 5.0):
       
        def _fn(r):
            # real mavutil object will send the command
            r.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm,
                0, 0, 0, 0, 0
            )

        try:
            if sync:
                # wait for send thread to accept and execute
                self.mavio.send_sync(_fn, timeout=timeout)
            else:
                # fire-and-forget: enqueue for the send thread
                self.mavio.send_async(_fn)
            self.messages.append(f"Servo {channel} set to {pwm}")
            # record last-sent PWM so we don't spam identical commands
            self._servo_last_sent_pwm = int(pwm)
        except Exception as e:
            self.messages.append(f"Failed to set servo {channel} -> {pwm}: {e}")

    def set_vel(self,vx=0,vy=0,vz=0):
        '''Target velocity in x,y,z'''
        time_boot_ms = int((time.time() - self.boot_time) * 1000)
        def _send_vel(r):
            r.mav.set_position_target_local_ned_send(
                time_boot_ms,  # time_boot_ms
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,  # Velocity only (vx,vy,vz)
                0, 0, 0,  # Position (unused)
                vx, vy, vz,  # Velocity
                0, 0, 0,  # Acceleration (unused)
                0, 0      # Yaw, yaw_rate (unused)
            )
        
        self.mavio.send_async(_send_vel)

    def set_pos(self,x,y,z):
        """Handle /set_position_local commands in NED frame"""
        
        # Send SET_POSITION_TARGET_LOCAL_NED
        def _send_local(r):
            r.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111111000,  # Position only (x,y,z)
                x, y, z,  # Position
                0, 0, 0,  # Velocity (unused)
                0, 0, 0,  # Acceleration (unused)
                0, 0      # Yaw, yaw_rate (unused)
            )
        
        self.mavio.send_async(_send_local)

    def set_alt(self,target_alt):
        '''Target Altitude in meters'''

        TYPE_MASK = 3579  #  Altitude only
        time_boot_ms = int((time.time() - self.boot_time) * 1000)
        def _send_alt(r):    
            r.mav.set_position_target_global_int_send(
            int(time_boot_ms,),
            self.system_id,
            self.component_id,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            TYPE_MASK,

            0, 0,               # ignored lat/lon
            float(target_alt),  # ? altitude used

            0, 0, 0,             # velocity ignored
            0, 0, 0,             # accel ignored
            0, 0                 # yaw ignored
        )
        self.mavio.send_async(_send_alt)

    def send_flow(self,x,y,quality,dt,xgyro,ygyro,zgyro,):
        def _sendflow(r):
            r.mav.optical_flow_rad_send(
            int(time.time() * 1e6),      # time_usec
            0,                           # sensor_id
            dt, #integration_time_us = dt * 1e6
            x,  #flow_rad = pixel_flow * (FOV / resolution)
            y,
            dt*xgyro,
            dt*ygyro,
            dt*zgyro,
            0,
            quality,
            0,
            self.state["altitude"]
            )
        self.mavio.send_async(_sendflow)

    def guided_setpoint(self):
        pass

    def plnd_target(self,angle_x,angle_y,distance,size_x=0,size_y=0):
        ''' Send LANDING_TARGET message angles are in radian'''
        def _send_land_target(r):
            r.mav.landing_target_send(
                int(time.time() * 1e6),  # time_usec
                0,  # target_num
                mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
                angle_x,  # angle_x (radians)
                angle_y,  # angle_y (radians)
                distance,  # distance (meters)
                size_x,  # size_x
                size_y   # size_y
            )
        
        self.mavio.send_async(_send_land_target)

    def upload_mission(self):
        pass

    def send_takeoff(self,alt):
        def _takeoff(r):
            r.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                0, 0,
                float(alt))
        self.mavio.send_async(_takeoff)

    def get_attitude(self):
        pass

    def get_altitude(self):
        return self.state["altitude"]

    def get_rc(self):
        pass


