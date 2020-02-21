import cv2, queue, threading, time

# bufferless VideoCapture
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    counter = 0
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
          counter -= 1
        except queue.Empty:
          pass
      self.q.put(frame)
      counter += 1
      print(f"size: {self.q.qsize()}, counter: {counter}")

  def read(self):
    return self.q.get()

cap = VideoCapture(0)
while True:
  time.sleep(1)   # simulate time between events
  frame = cap.read()
  cv2.imshow("slow", frame)
  print("grabbing")
  if chr(cv2.waitKey(1)&255) == 'q':
    break