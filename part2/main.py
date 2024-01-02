import asyncio
from bleak import BleakScanner, BleakClient
import struct
import sys
import os
import time
import threading

# Scan through all peripherals
async def discover():
    devices = await BleakScanner.discover()

    for d in devices:
        print(d)

# Asynchronous event handler
loop = asyncio.get_event_loop()

# Scan all peripherals
loop.run_until_complete(discover())

# After scanning, change the address to the appropriate value
# The Arduino device will be displayed with locally assigned name
address0 = "E1:B1:B5:DF:56:15" # set your address here
address1 = "A6:E2:E4:56:59:23"

# Standard BLE/Bluetooth UUID is 128-bit long
# Among them, below formats are specified/reserved by the standard 16-bit UUID assignments
uuid_extend = lambda uuid_16: "0000" + uuid_16 + "-0000-1000-8000-00805f9b34fb"

    
# -------------------------------------------------------------------------
# global variables
base_times = [0, 0]
last_time = 0
last_hand = 0
last_interval_time = 0
hit_cnts = [0, 0]
hit_history = []
weighted_avg_bpm = 0
total_hit_cnt = 0
interval_idx = 0
done = 0

cali_num = 2
cali_cnt = [0, 0]
last_cali_time = [0, 0]
cali_done = [0, 0]
time_offs = [0, 0]

# scores
score_lr_harmony = 0           # 0~100
score_const_speed = 0           # 0~100
score_const_loud = 0           # 0~100
discard_first_cnt = 4
discard_last_cnt = 4

# --------------------------------------------------------------------------
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
        else:
            sys.exit()
    
    def stop(self):
        self.done=True

# def getkey():
#     fd = sys.stdin.fileno()
#     or_attributes = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, or_attributes)
#     return ch

# --------------------------------------------------------------------------
def PrintStatistics():
    global base_times
    global last_time
    global done
    global hit_history
    global total_hit_cnt
    global hit_cnts
    global interval_idx
    global last_interval_time
    global weighted_avg_bpm
    
    # hand = 0
    ENDTIME_THRESHOLD = 3000
    MINIMUM_HIT_CNT = 5

    if last_time - base_times[0] == 0:
        return

    hit_interval = hit_history[total_hit_cnt-1][0] - hit_history[total_hit_cnt-2][0]
    if last_time != 0 and \
       total_hit_cnt > MINIMUM_HIT_CNT and \
       hit_interval > ENDTIME_THRESHOLD:
        done = 1
        PrintScore()
        return
    
    # only print when minimum data is gathered
    interval_hit_cnt = total_hit_cnt - interval_idx
    if interval_hit_cnt < 4:
        return

    # calculate bpm
    avg_bpm = 15000*(total_hit_cnt-1)/(last_time - hit_history[0][0])
    bpm_weight = 0.2

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
    os.system("cls")
    print('Stats')
    print('avg_bgm {}, weighted_avg_bpm {}, hit_cnts {}, total_hit_cnt {}'.format(avg_bpm, weighted_avg_bpm, hit_cnts, total_hit_cnt))
    print('interval_bgm {}, bpm_diff {}'.format(interval_bpm, interval_bpm - weighted_avg_bpm))
    sys.stdout.flush()

    interval_idx = total_hit_cnt - 1
    
# --------------------------------------------------------------------------
def PrintConstSpeed():
    global score_const_speed
    global total_hit_cnt
    global hit_history
    global discard_first_cnt
    global discard_last_cnt

    SCALING_FACTOR = 10
    ERROR_OFFSET = 0.2
    FAST_STROKE_BONUS = 0.03

    speed_variable_score = 0
    interval_speed_variable = 0

    min_interval_cnt = 0
    interval_len = 5

    hit_cnt = total_hit_cnt - discard_first_cnt - discard_last_cnt - 1

    # debug
    # print(hit_history)

    if total_hit_cnt <= min_interval_cnt + interval_len + discard_first_cnt + discard_last_cnt:
        print('sample is too few, please practice longer!')
        return

    # calc base bpm
    first_time = hit_history[discard_first_cnt][0]
    last_time = hit_history[total_hit_cnt - 1 - discard_last_cnt][0]
    
    avg_bpm = 15000*(hit_cnt - 1)/(last_time - first_time)
    # print('avg_bpm: {}'.format(avg_bpm))

    # calc error
    bpm_err = 0
    i = discard_first_cnt
    while i < total_hit_cnt - discard_last_cnt - interval_len:
        interval_bpm = 15000*(interval_len-1)/(hit_history[i+interval_len-1][0] - hit_history[i][0])
        interval_bpm_err = abs(avg_bpm - interval_bpm)/(avg_bpm ** (1+FAST_STROKE_BONUS))
        bpm_err += interval_bpm_err
        i += 1
        # print('interval_bpm: {}, raw_err: {}, error: {}'.format(interval_bpm, \
        #                                                         abs(avg_bpm - interval_bpm), \
        #                                                         interval_bpm_err))

    bpm_err *= SCALING_FACTOR
    bpm_err /= hit_cnt - interval_len + 1
    
    score_const_speed = 1.2 ** (0 - bpm_err + ERROR_OFFSET)
    score_const_speed *= 100
    if score_const_speed > 100:
        score_const_speed = 100
    
    # print('overall bpm_err: {}, score_const_speed: {}'.format(bpm_err, score_const_speed))
    # print('raw score: {}'.format(1.2**(0-bpm_err)))

    print('* Const Speed Score: {}'.format(score_const_speed))

    
# --------------------------------------------------------------------------
def PrintLRHarmony():
    global score_lr_harmony
    global total_hit_cnt
    global hit_history
    global discard_first_cnt
    global discard_last_cnt

    SCALING_FACTOR = 5
    ERROR_OFFSET = 0.2
    FAST_STROKE_BONUS = 0.03

    SAME_HAND_PANELTY = 1
    
    speed_variable_score = 0
    interval_speed_variable = 0

    min_interval_cnt = 0
    interval_len = 3            # it should be more than 3

    hit_cnt = total_hit_cnt - discard_first_cnt - discard_last_cnt - 1
    
    if total_hit_cnt <= min_interval_cnt + interval_len + discard_first_cnt + discard_last_cnt:
        print('sample is too few, please practice longer!')
        return

    # measure if the time of 'L' is middle between two 'R', in RLR, and vice versa
    harmony_err = 0
    i = discard_first_cnt
    while i < total_hit_cnt - discard_last_cnt - interval_len:
        # verify if hit was LRLR.., add penalty if not
        if hit_history[i][1] == hit_history[i+1][1] or hit_history[i+1][1] == hit_history[i+2][1]:
            harmony_err += SAME_HAND_PANELTY
            i += 1
            continue
        
        rr_interval = hit_history[i+2][0] - hit_history[i][0]
        rl_interval = hit_history[i+1][0] - hit_history[i][0]
        harmony_err += abs(rr_interval - rl_interval*2)/(rr_interval)
        i += 1
        # print('harmony_err: {}, rr_interval: {}, 2*rl_interval: {}'.format(harmony_err, \
        #                                                                    rr_interval, \
        #                                                                    rl_interval*2))

    harmony_err *= SCALING_FACTOR
    harmony_err /= hit_cnt - interval_len + 1
    
    score_lr_harmony = 1.2 ** (0 - harmony_err + ERROR_OFFSET)
    score_lr_harmony *= 100
    if score_lr_harmony > 100:
        score_lr_harmony = 100
    
    # print('overall harmony_err: {}, score_lr_harmony: {}'.format(harmony_err, score_lr_harmony))
    # print('raw score: {}'.format(1.2**(0-harmony_err)))

    print('* LR Harmony Score: {}'.format(score_lr_harmony))

    
# --------------------------------------------------------------------------
def PrintConstLoudness():
    global score_const_loud
    global total_hit_cnt
    global hit_history
    global discard_first_cnt
    global discard_last_cnt

    SCALING_FACTOR = 10
    ERROR_OFFSET = 0.2

    speed_variable_score = 0
    interval_speed_variable = 0

    min_interval_cnt = 0

    LOUD_THRESHOLD = 0.2

    hit_cnt = total_hit_cnt - discard_first_cnt - discard_last_cnt - 1

    # get avg loudness
    avg_loud = 0
    i = discard_first_cnt
    while i < total_hit_cnt - discard_last_cnt:
        avg_loud += hit_history[i][2]
        i += 1
    avg_loud /= hit_cnt

    # print('avg_loud: {}'.format(avg_loud))

    # get error
    loud_err = 0
    i = discard_first_cnt
    while i < total_hit_cnt - discard_last_cnt:
        tmp_err = abs(avg_loud - hit_history[i][2])/avg_loud
        if tmp_err > LOUD_THRESHOLD:
            loud_err += tmp_err
        i += 1
        # print('tmp_err: {}, loud_err: {}'.format(tmp_err, \
        #                                          loud_err))

    loud_err *= SCALING_FACTOR
    loud_err /= hit_cnt 
    
    score_const_loud = 1.5 ** (0 - loud_err + ERROR_OFFSET)
    score_const_loud *= 100
    if score_const_loud > 100:
        score_const_loud = 100
    
    # print('overall loud_err: {}, score_const_loud: {}'.format(loud_err, score_const_loud))
    # print('raw score: {}'.format(1.5**(0-loud_err)))

    print('* Const Loudness Score: {}'.format(score_const_loud))

    
# --------------------------------------------------------------------------
def PrintScore():
    global timer
    os.system('cls')

    # preprocess
    sorted(hit_history, key=lambda hit_log: hit_log[0])
    
    # print score
    print('Overall Skill Score:')
    PrintConstSpeed()
    PrintLRHarmony()
    PrintConstLoudness()
    
    timer.done = True
# --------------------------------------------------------------------------
def analyze(time, hand, strength):
    
    global base_times
    global last_time
    global done
    global total_hit_cnt
    global hit_cnts
    global last_hand

    ADD_THRESHOLD = 50
    
    if done:
        return
    
    # end program when there is long empty time
    hit_interval = time - last_time

    # filter some over-sensed one
    if last_hand == hand and hit_interval < ADD_THRESHOLD:
        # debug
        print('scratch is happened, hit_interval: {}'.format(hit_interval))
        
        return
    
    # update states
    last_time = time
    last_hand = hand
    total_hit_cnt = total_hit_cnt + 1
    hit_history.append([time, hand, strength])
    hit_cnts[hand] = hit_cnts[hand] + 1
    
    instant_bpm = 15000/hit_interval
    if total_hit_cnt > 5:
        avg_bpm = 15000*(total_hit_cnt-1)/(time - hit_history[0][0])
    else:
        avg_bpm = 0
    
    # debug - print stats
    print(time, strength, hand)
    
    print(hit_interval, instant_bpm, avg_bpm, hit_cnts[hand], total_hit_cnt)


# --------------------------------------------------------------------------
# Simple print callback for the "notify" event of the "Intermediate Temperature Characteristic"
def intermediate_hit_callback0(sender, data):
    my_struct = struct.unpack('lf', data[1:9])
    time_ms = round(my_struct[0] / 1000)
    # capture offset calibration at first..
    if cali_done[0] == 0:
        if time_ms - last_cali_time[0] > 250:
            print("First calibrate time offset, please hit two stick each other {} times!".format(3 - cali_cnt[0]))
            time_offs[0] += time_ms
            last_cali_time[0] = time_ms
            print("time off {}: {}".format(0, time_offs[0]))
            cali_cnt[0] += 1
        if cali_cnt[0] >= cali_num:
            cali_done[0] = 1
        return
    if cali_done[0] == 1:
        time_offs[0] = round(time_offs[0] / cali_cnt[0], 3)
        cali_done[0] = 2

    analyze(time_ms - time_offs[0], 0, my_struct[1])

    # print("id : 0 timing: {} acc: {}".format(*struct.unpack('lf', data[1:9]))) # Field struct is 9-byte long, where the first byte is the "flag" field.

def intermediate_hit_callback1(sender, data):
    my_struct = struct.unpack('lf', data[1:9])
    time_ms = round(my_struct[0] / 1000)
    # capture offset calibration at first..
    if cali_done[1] == 0:
        if time_ms - last_cali_time[1] > 250:
            print("First calibrate time offset, please hit two stick each other {} times!".format(3 - cali_cnt[1]))
            time_offs[1] += time_ms
            last_cali_time[1] = time_ms
            print("time off {}: {}".format(0, time_offs[1]))
            cali_cnt[1] += 1
        if cali_cnt[1] >= cali_num:
            cali_done[1] = 1
        return
    if cali_done[1] == 1:
        time_offs[1] = round(time_offs[1] / cali_cnt[1], 3)
        cali_done[1] = 2

    analyze(time_ms - time_offs[1], 1, my_struct[1])

    # print("id : 1 timing: {} acc: {}".format(*struct.unpack('lf', data[1:9]))) # Field struct is 9-byte long, where the first byte is the "flag" field.

# --------------------------------------------------------------------------
# Main asynchronous function, with target address
async def run(address0, address1):
    print("Trying to connect...")
    async with BleakClient(address0, use_cached=False) as client0:
        print("1. Connected to {}".format(address0))

        async with BleakClient(address1, use_cached=False) as client1:
            print("2. Connected to {}".format(address1))

            await client0.start_notify(uuid_extend("2A1E"), intermediate_hit_callback0)
            await client1.start_notify(uuid_extend("2A1E"), intermediate_hit_callback1)
            # Make the main function loop forever to continuously monitor the data
            while client0.is_connected and client1.is_connected:
                await asyncio.sleep(1)

timer = Timer_alarm(increment = 1)
loop.run_until_complete(run(address0, address1))
