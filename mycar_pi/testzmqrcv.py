import zmq
import time

context = zmq.Context()
sock = context.socket(zmq.SUB)
sock.connect("tcp://192.168.1.101:5556")
sock.setsockopt_string(zmq.SUBSCRIBE, '')

while True:
  payload = sock.recv().decode("utf-8")
  print(payload)
  time.sleep(1)

