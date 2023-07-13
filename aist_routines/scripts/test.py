import nep
import time

# Create a new nep node
node = nep.node("receiver")
# Important: 	You need to change the IP address <'127.0.0.1'> by
# 		the IP address of PC running NEP CLI

conf = node.hybrid('163.220.51.108')
# Create a new nep subscriber with the topic <'test'>
sub = node.new_sub('test', "json", conf)

while True:
    # Read data in a non-blocking mode
    s, msg = sub.listen()
    # if s == True, then there is data in the socket
    if s:
        print(msg)
    else:
        # An small sleep will reduce computational load
        time.sleep(.0001)
