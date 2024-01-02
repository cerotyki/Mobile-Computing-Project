import sys
import os
import tty
import termios
import time
import threading


class Timer_alarm():
    def __init__(self, increment):
        self.next_t = time.time()
        self.i=0
        self.done=False
        self.increment = increment
        self._run()

    def _run(self):
        PrintStatistics()
        self.next_t+=self.increment
        self.i+=1
        if not self.done:
            threading.Timer( self.next_t - time.time(), self._run).start()
    
    def stop(self):
        self.done=True

def getkey():
    fd = sys.stdin.fileno()
    or_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, or_attributes)
    return ch

def PrintStatistics():
    global base_times
    global last_time
    global done
    global total_hit_cnt
    global hit_cnts
    global interval_idx
    global last_interval_time
    global weighted_avg_bpm
    
    # hand = 0
    if last_time - base_times[0] == 0:
        print('last_time - base_times[0] = 0')
        return

    # only print when minimum data is gathered
    interval_hit_cnt = total_hit_cnt - interval_idx
    if interval_hit_cnt < 4:
        return

    # calculate bpm
    avg_bpm = 15000*(total_hit_cnt-1)/(last_time - base_times[0])
    bpm_weight = 0.5

    if last_interval_time == 0:
        interval_bpm = avg_bpm
        last_interval_time = last_time
    else:
        interval_bpm = 15000*(interval_hit_cnt - 1)/(last_time - last_interval_time)
        last_interval_time = last_time

    if weighted_avg_bpm == 0:
        weighted_avg_bpm = avg_bpm
    else:
        weighted_avg_bpm = interval_bpm * bpm_weight + weighted_avg_bpm * (1 - bpm_weight)

    # print main statistics
    os.system('clear')
    print('Stats')
    print('avg_bgm {}, weighted_avg_bpm {}, hit_cnts {}, total_hit_cnt {}'.format(avg_bpm, weighted_avg_bpm, hit_cnts, total_hit_cnt))
    print('interval_bgm {}, bpm_diff {}'.format(interval_bpm, interval_bpm - weighted_avg_bpm))
    sys.stdout.flush()


    interval_idx = total_hit_cnt - 1
    
        
def analyze(time, hand, strength):
    LEFT  = 0
    RIGHT = 1
    ENDTIME_THRESHOLD = 5000
    
    global base_times
    global last_time
    global done
    global total_hit_cnt
    global hit_cnts

    # end program when there is long empty time
    hit_interval = time - last_time
    if last_time != 0 and hit_interval > ENDTIME_THRESHOLD:
        done = 1
        return

    # update states
    last_time = time
    total_hit_cnt = total_hit_cnt + 1
    hit_history.append([time, hand, strength]);
    hit_cnts[hand] = hit_cnts[hand] + 1
    
    # calculate base stat
    if base_times[hand] == 0:
        base_times[hand] = time
        print("hand {}'s base time set to {}".format(hand, time));
        return
    
    instant_bpm = 15000/hit_interval
    avg_bpm = 15000*(total_hit_cnt-1)/(time - base_times[hand])
    
    # debug - print stats
    print(time, strength, hand);
    
    # print(hit_interval, instant_bpm, avg_bpm, hit_cnts[hand], total_hit_cnt)


def main():
    time_offs = [0, 0]
    hand = 0                    # 0:left   1: right
    print('Simple simulation of host as drum skill analyzer!');


    # capture offset calibration at first..
    print('First calibrate time offset, please hit two stick each other 3 times!');
    cali_cnt = [0, 0]
    last_cali_time = [0, 0]
    cali_done = 0
    while not cali_done:
        c = getkey()
        if c == 'f':
            hand = 0
        elif c == 'q':
            a.stop()
            sys.exit()
        else:
            hand = 1
        
        time_ms = round(time.time() * 1000)

        if time_ms - last_cali_time[hand] > 250:
            time_offs[hand] += time_ms
            last_cali_time[hand] = time_ms
            print("time off {}: {}".format(hand, time_offs[hand]))
            cali_cnt[hand] += 1
        if cali_cnt[0] >= 3 and cali_cnt[1] >= 3:
            cali_done = 1
            
    time_offs[0] /= cali_cnt[0]
    time_offs[1] /= cali_cnt[1]
    
    print('calibration done: {} {}'.format(time_offs[0], time_offs[1]));
    print('please press \'f\' and \'j\', simulating every \'beat\'');

    global done

    a=Timer_alarm(increment = 1)
    
    while(not done):
        c = getkey()
        if c == 'f':
            hand = 0
        elif c == 'q':
            a.stop()
            sys.exit()
        else:
            hand = 1
        
        time_ms = round(time.time() * 1000)

        # capture offset calibration at first..
        # uncomment this when port this to real drum stick application
        # if time_offs[hand]  == 0:
        #     time_offs[hand] = time_ms
        #     print("time off {}: {}".format(hand, time_offs[hand]))
        #     continue

        analyze(time_ms - time_offs[hand], hand, 5)
    
# global variables
base_times = [0, 0]
last_time = 0
last_interval_time = 0
hit_cnts = [0, 0]
hit_history = []
weighted_avg_bpm = 0
total_hit_cnt = 0
interval_idx = 0
done = 0

main()
