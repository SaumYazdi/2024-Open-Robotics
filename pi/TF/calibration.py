import cv2 as cv
from os.path import join, dirname
from ball import BallDetector
from camera import Camera

SPACE = ord(' ')

def calibration_test():
    cam = Camera()
    detector = BallDetector('transformed_frozen_inference_graph.pb', 'ssd_mobilenet_v1_balls.pbtxt')

    # FOV: horizontal=62.2 vertical=48.8

    measured = []

    angle = -30

    while(True):
        ret, frame = cam.video.read() 
        boxes = detector.detect_frame(frame)
        cam.draw_boxes(frame, boxes)
    
        cv.imshow('Camera', frame)
        
        keycode = cv.waitKey(1) & 0xFF
        if keycode == ord('q'): 
            break
    
        elif keycode == SPACE: 
            if angle <= 30:
                print(f"Current angle: {angle}")

                if len(boxes) > 0:

                    ball_bbox = sorted(boxes, key=lambda box: (box[0] - box[2]) * (box[1] - box[3]))[-1]
                    center_x = ball_bbox.center[0]

                    measured.append((angle, center_x))

                    angle += 10

            if angle >  30:
                print("Finished")

                with open(join(dirname(__file__), "out.txt"), "w") as f:
                    s = ""
                    for row in measured:
                        s += f"{row[0]} {row[1]}\n"
                    f.write(s)

                break

    cam.video.release()
    cv.destroyAllWindows() 


if __name__ == "__main__":
    calibration_test()