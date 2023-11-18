from threading import Thread
import time

text = "I love MSE4499"

# class first(Thread): 
#     def run(self): 
#         for c in text: 
#             time.sleep(1)
#             print(c)

# class second(Thread): 
#     def run(self): 
#         time.sleep(5)
#         print("5 Seconds Up")

# first().start()
# second().start()



for c in text: 
    time.sleep(1)
    print(c)

time.sleep(5)
print("5 Seconds Up")

