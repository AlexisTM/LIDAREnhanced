#!/usr/bin/env python

from threading import Thread
import time


def sender():
    global z
    for i in range(1, 11):
        print(z)
        time.sleep(1)

def increment():
    global z
    for i in range(1, 11):
        z = z + 1
        time.sleep(1)

def main():
    global z
    z = 0
    tinc = Thread(target=increment).start()
    tsender = Thread(target=sender).start()
    #time.sleep(10)

main()