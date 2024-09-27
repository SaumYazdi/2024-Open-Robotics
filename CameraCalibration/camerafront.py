
class FrontFacingCamera:
    def __init__(self, window_name: str,
            preview: bool = False, draw_detections: bool = False,
            camera_port: int = 1):
                
        if DEVICE == "pi":
            self.video_stream = Picamera2(camera_port)
            raw_config = CAMERA_SENSOR_MODES[1]
            raw_config["fps"] = 60
            config = self.video_stream.create_video_configuration(
                main={"format": 'XRGB8888', "size": [640, 480]},
                raw=raw_config,
                buffer_count=6,
                controls={'FrameRate': 60},
            )
            self.video_stream.configure(config)

        elif DEVICE == "pc":
            self.video_stream = VideoStream(src=0).start()

        self.window_name = window_name
        self.preview = preview
        self.draw_detections = draw_detections
        self.fps = None
                
        self.pos = None
        self.radius = None

        self.distance = None
        self.angle = None

        self.image = None
        self.color_mask = None

        self.update_events = []
        self.save_path = join(dirname(__file__), "save-front.json")
        
        self.set_color_bounds(color["lower"], color["upper"])
        
    def set_color_bounds(self, lower, upper):
        self.orange_lower = tuple(lower)
        self.orange_upper = tuple(upper)
    
    def get_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        self.color_mask = hsv.copy()
        
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        return mask

    def get_distance(self):
        if self.radius is None:
            return None
        try:
            with open(self.save_path, "r") as f:
                data = json.load(f)
                dist = data["dist"]
        except json.JSONDecodeError:
            return self.distance
        return dist["k"] * pow(self.radius, dist["a"])


    def get_angle(self, distance = None):
        if self.radius is None:
            return None
        try:
            with open(self.save_path, "r") as f:
                data = json.load(f)
                angle = data["angle"]

            if distance is None:
                distance = self.get_distance()
            normal_x = self.pos.x / distance
            estimated_x = angle["m"] * normal_x + angle["c"]
            return atan2(estimated_x, distance)
    
        except json.JSONDecodeError:
            return self.angle
                
    
    def compute(self, frame):
        mask = self.get_mask(frame)
        
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        
        if len(contours) > 0:
            
            # Merge contours into a convex contour to be fitted with ellipse
            points = []
            
            # Loop over contours from largest to smallest area
            sorted_contours = sorted(contours, key=lambda x: x.size, reverse=True)
            pX = pY = None
            
            for i in range(len(sorted_contours)):
                cnt = sorted_contours[i]
                
                # Skip over contour if size is too small
                if cnt.size < 20:
                    continue
                    
                # Compute center of contour
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if pX is None:
                    dist_to_last_contour = 0
                else:
                    dist_to_last_contour = sqrt((cX - pX)**2 + (cY - pY)**2)
                    
                # If the distance to the last (valid) contour is too far, then skip
                if dist_to_last_contour > 120:
                    continue
                pX, pY = cX, cY
                if self.draw_detections:
                    cv2.drawMarker(frame, (cX, cY), (255, 0, 0))
                for pt in cnt:
                    points.append(pt)
            
            if len(points) > 4:
                c = cv2.convexHull(numpy.array(points, dtype=numpy.int32))
                
                if c.size > 4:
                    ellipse = cv2.fitEllipse(c)
                    center, size, angle = ellipse
                    
                    # Not actually radius, i just cbf changing the variable name
                    self.pos = Vector(center)
                    self.radius = size[0] * size[1]
                    try:
                        self.distance = self.get_distance()
                        self.angle = self.get_angle(self.distance)
                    except PermissionError:
                        pass
                    scale = size[0] * .006
                    
                    if self.draw_detections:
                        
                        for i in range(len(contours)):
                            cv2.drawContours(frame, contours, i, (0, 255, 0))
                            
                        cv2.ellipse(frame, ellipse, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.drawMarker(frame, [int(center[0]), int(center[1])], (0, 255, 0))

                if self.radius > MIN_RADIUS:
                    if self.draw_detections:
                        # Draw True Circle Pos and Centroid
                        # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        # cv2.circle(frame, center, 2, BALL_TRACK_COLOUR, -1)
                        scale = size[1] * .006
                        
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        text_pos = [int(self.pos.x - 140 * scale), int(self.pos.y - 25 * scale)]
                        thickness = 1
                        cv2.putText(frame, f"dist: {self.distance:.2f}cm", text_pos, font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
                        cv2.putText(frame, f"angle: {degrees(self.angle):.2f} deg", (text_pos[0], int(text_pos[1] + 50 * scale)), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)

        else:
            self.pos = None
            self.radius = None
            self.distance = None
            self.angle = None

        # self.points.appendleft(self.pos.int() if self.pos else None)
        return frame
    
    def read(self):
        if DEVICE == "pc":
            return self.video_stream.read()
        return self.video_stream.capture_array()
    
    def _update(self):
        # start = perf_counter()
        frame = self.read()
        # print(f"'read frame' took {(perf_counter() - start) * 1000:.2f}ms")
        
        if frame is None:
            return
        
        # Important or detection will break!
        # Resize to detection resolution
        if type(RESIZE_WIDTH) == int:
            frame = imutils.resize(frame, width=RESIZE_WIDTH)
        frame = self.compute(frame)

        if self.draw_detections:
            # Draw Ball Trail
            # for i in range(1, len(self.points)):
                # if self.points[i - 1] is None or self.points[i] is None:
                    # continue
                
                # thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
                # cv2.line(frame, self.points[i - 1], self.points[i], BALL_TRACK_COLOUR, thickness)
            
            if self.pos and self.radius:
                cv2.circle(frame, self.pos.int(), int(self.radius), BALL_TRACK_COLOUR, 1, cv2.LINE_AA)

                # Draw Velocity Vector
                # point2 = Vector(self.pos.x + self.velocity.x * 50, self.pos.y + self.velocity.y * 50)
                # cv2.line(frame, self.pos.int(), point2.int(), (255, 0, 50), 3)

        if self.preview == True:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                return
        
        return frame

    def set_update(self, event: Callable):
        self.update_events.append(event)

    def start(self, delay=2) -> None:
        if DEVICE == "pi":
            self.video_stream.start()
                
            sleep(delay)
            
            count = 10
            start = perf_counter()
            for i in range(0,count) :
                request = self.video_stream.capture_request()
                metadata = request.get_metadata()
                request.release()
            fps = float(count/(perf_counter() - start))
            print("Metadata only", " Spf:", 1./fps, " Fps:", fps)
            
        ticks = 1
        prev = perf_counter()
        while True:
            if (ticks % FPS_UPDATE) == 0:
                now = perf_counter()
                dt = now - prev
                prev = now
                self.fps = FPS_UPDATE / dt 

                if self.preview:
                    cv2.setWindowTitle(self.window_name, f"FPS: {self.fps}")

            self.image = self._update()
            if self.image is None:
                break

            for event in self.update_events:
                event()
            
            ticks += 1

        self.stop()

    def stop(self) -> None:
        if DEVICE == "pi":
            self.video_stream.close()
        elif DEVICE == "pc":
            self.video_stream.stop()
        cv2.destroyAllWindows()






class FrontFacingCamera:
    def __init__(self, window_name: str,
            preview: bool = False, draw_detections: bool = False,
            camera_port: int = 1):
                
        if DEVICE == "pi":
            self.video_stream = Picamera2(camera_port)
            raw_config = CAMERA_SENSOR_MODES[1]
            raw_config["fps"] = 60
            config = self.video_stream.create_video_configuration(
                main={"format": 'XRGB8888', "size": [640, 480]},
                raw=raw_config,
                buffer_count=6,
                controls={'FrameRate': 60},
            )
            self.video_stream.configure(config)

        elif DEVICE == "pc":
            self.video_stream = VideoStream(src=0).start()

        self.window_name = window_name
        self.preview = preview
        self.draw_detections = draw_detections
        self.fps = None
                
        self.pos = None
        self.radius = None

        self.distance = None
        self.angle = None

        self.image = None
        self.color_mask = None

        self.update_events = []
        self.save_path = join(dirname(__file__), "save-front.json")
        
        self.set_color_bounds(color["lower"], color["upper"])
        
    def set_color_bounds(self, lower, upper):
        self.orange_lower = tuple(lower)
        self.orange_upper = tuple(upper)
    
    def get_mask(self, frame):
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        self.color_mask = hsv.copy()
        
        print(self.orange_upper)
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        return mask

    def get_distance(self):
        if self.radius is None:
            return None
        try:
            with open(self.save_path, "r") as f:
                data = json.load(f)
                dist = data["dist"]
        except json.JSONDecodeError:
            return self.distance
        return dist["k"] * pow(self.radius, dist["a"])


    def get_angle(self, distance = None):
        if self.radius is None:
            return None
        try:
            with open(self.save_path, "r") as f:
                data = json.load(f)
                angle = data["angle"]

            if distance is None:
                distance = self.get_distance()
            normal_x = self.pos.x / distance
            estimated_x = angle["m"] * normal_x + angle["c"]
            return atan2(estimated_x, distance)
    
        except json.JSONDecodeError:
            return self.angle
                
    
    def compute(self, frame):
        mask = self.get_mask(frame)
        
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        
        if len(contours) > 0:
            
            # Merge contours into a convex contour to be fitted with ellipse
            points = []
            
            # Loop over contours from largest to smallest area
            sorted_contours = sorted(contours, key=lambda x: x.size, reverse=True)
            pX = pY = None
            
            for i in range(len(sorted_contours)):
                cnt = sorted_contours[i]
                
                # Skip over contour if size is too small
                if cnt.size < 20:
                    continue
                    
                # Compute center of contour
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if pX is None:
                    dist_to_last_contour = 0
                else:
                    dist_to_last_contour = sqrt((cX - pX)**2 + (cY - pY)**2)
                    
                # If the distance to the last (valid) contour is too far, then skip
                if dist_to_last_contour > 120:
                    continue
                pX, pY = cX, cY
                if self.draw_detections:
                    cv2.drawMarker(frame, (cX, cY), (255, 0, 0))
                for pt in cnt:
                    points.append(pt)
            
            if len(points) > 4:
                c = cv2.convexHull(numpy.array(points, dtype=numpy.int32))
                
                if c.size > 4:
                    ellipse = cv2.fitEllipse(c)
                    center, size, angle = ellipse
                    
                    # Not actually radius, i just cbf changing the variable name
                    self.pos = Vector(center)
                    self.radius = size[0] * size[1]
                    try:
                        self.distance = self.get_distance()
                        self.angle = self.get_angle(self.distance)
                    except PermissionError:
                        pass
                    scale = size[0] * .006
                    
                    if self.draw_detections:
                        
                        for i in range(len(contours)):
                            cv2.drawContours(frame, contours, i, (0, 255, 0))
                            
                        cv2.ellipse(frame, ellipse, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.drawMarker(frame, [int(center[0]), int(center[1])], (0, 255, 0))

                if self.radius > MIN_RADIUS:
                    if self.draw_detections:
                        # Draw True Circle Pos and Centroid
                        # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        # cv2.circle(frame, center, 2, BALL_TRACK_COLOUR, -1)
                        scale = size[1] * .006
                        
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        text_pos = [int(self.pos.x - 140 * scale), int(self.pos.y - 25 * scale)]
                        thickness = 1
                        cv2.putText(frame, f"dist: {self.distance:.2f}cm", text_pos, font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
                        cv2.putText(frame, f"angle: {degrees(self.angle):.2f} deg", (text_pos[0], int(text_pos[1] + 50 * scale)), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)

        else:
            self.pos = None
            self.radius = None
            self.distance = None
            self.angle = None

        # self.points.appendleft(self.pos.int() if self.pos else None)
        return frame
    
    def read(self):
        if DEVICE == "pc":
            return self.video_stream.read()
        return self.video_stream.capture_array()
    
    def _update(self):
        # start = perf_counter()
        frame = self.read()
        # print(f"'read frame' took {(perf_counter() - start) * 1000:.2f}ms")
        
        if frame is None:
            return
        
        # Important or detection will break!
        # Resize to detection resolution
        if type(RESIZE_WIDTH) == int:
            frame = imutils.resize(frame, width=RESIZE_WIDTH)
        frame = self.compute(frame)

        if self.draw_detections:
            # Draw Ball Trail
            # for i in range(1, len(self.points)):
                # if self.points[i - 1] is None or self.points[i] is None:
                    # continue
                
                # thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
                # cv2.line(frame, self.points[i - 1], self.points[i], BALL_TRACK_COLOUR, thickness)
            
            if self.pos and self.radius:
                cv2.circle(frame, self.pos.int(), int(self.radius), BALL_TRACK_COLOUR, 1, cv2.LINE_AA)

                # Draw Velocity Vector
                # point2 = Vector(self.pos.x + self.velocity.x * 50, self.pos.y + self.velocity.y * 50)
                # cv2.line(frame, self.pos.int(), point2.int(), (255, 0, 50), 3)

        if self.preview == True:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                return
        
        return frame

    def set_update(self, event: Callable):
        self.update_events.append(event)

    def start(self, delay=2) -> None:
        if DEVICE == "pi":
            self.video_stream.start()
                
            sleep(delay)
            
            count = 10
            start = perf_counter()
            for i in range(0,count) :
                request = self.video_stream.capture_request()
                metadata = request.get_metadata()
                request.release()
            fps = float(count/(perf_counter() - start))
            print("Metadata only", " Spf:", 1./fps, " Fps:", fps)
            
        ticks = 1
        prev = perf_counter()
        while True:
            if (ticks % FPS_UPDATE) == 0:
                now = perf_counter()
                dt = now - prev
                prev = now
                self.fps = FPS_UPDATE / dt 

                if self.preview:
                    cv2.setWindowTitle(self.window_name, f"FPS: {self.fps}")

            self.image = self._update()
            if self.image is None:
                break

            for event in self.update_events:
                event()

            # except Exception as exc:
                # print(exc)
            
            ticks += 1

        self.stop()

    def stop(self) -> None:
        if DEVICE == "pi":
            self.video_stream.close()
        elif DEVICE == "pc":
            self.video_stream.stop()
        cv2.destroyAllWindows()
