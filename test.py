import tango
import numpy as np
import json
import os
import time
import matplotlib.pyplot as plt

mot = tango.DeviceProxy('B318A-test/PiezoMotorPMD/pmd301_ctrl_1')

prev_meas_200_Hz_up = {0: 3136, 379: 2704, 759: 2704, 1138: 2688, 1517: 2688, 1897: 2688, 2276: 2688, 2655: 2656, 3034: 2688, 3414: 2688, 3793: 2688, 4172: 2656, 4552: 2704, 4931: 2720, 5310: 2656, 5690: 2640, 6069: 2608, 6448: 2528, 6828: 2576, 7207: 1264, 7586: 1248, 7966: 1248, 8345: 1224, 8724: 1224, 9103: 1240, 9483: 1208, 9862: 1208, 10241: 1192, 10621: 1184, 11000: 3472}
prev_meas_500_Hz_up = {0: 3088, 379: 2688, 759: 2672, 1138: 2672, 1517: 2704, 1897: 2704, 2276: 2704, 2655: 2672, 3034: 2688, 3414: 2672, 3793: 2656, 4172: 2656, 4552: 2720, 4931: 2720, 5310: 2688, 5690: 2640, 6069: 2608, 6448: 2528, 6828: 2576, 7207: 1264, 7586: 1264, 7966: 1248, 8345: 1224, 8724: 1224, 9103: 1232, 9483: 1192, 9862: 1216, 10241: 1216, 10621: 1168, 11000: 3488}
prev_meas_500_Hz_up_100x = {0: 3120, 111: 2784, 222: 2720, 333: 2704, 444: 2688, 556: 2704, 667: 2688, 778: 2688, 889: 2688, 1000: 2688, 1111: 2672, 1222: 2672, 1333: 2688, 1444: 2688, 1556: 2704, 1667: 2704, 1778: 2672, 1889: 2608, 2000: 2640, 2111: 2640, 2222: 2688, 2333: 2704, 2444: 2688, 2556: 2688, 2667: 2672, 2778: 2656, 2889: 2656, 3000: 2688, 3111: 2688, 3222: 2704, 3333: 2656, 3444: 2672, 3556: 2672, 3667: 2688, 3778: 2688, 3889: 2688, 4000: 2672, 4111: 2672, 4222: 2688, 4333: 2688, 4444: 2688, 4556: 2704, 4667: 2720, 4778: 2688, 4889: 2720, 5000: 2704, 5111: 2688, 5222: 2704, 5333: 2656, 5444: 2608, 5556: 2560, 5667: 2608, 5778: 2592, 5889: 2592, 6000: 2592, 6111: 2576, 6222: 2576, 6333: 2576, 6444: 2528, 6556: 2512, 6667: 2544, 6778: 2560, 6889: 2528, 7000: 1264, 7111: 1256, 7222: 1264, 7333: 1248, 7444: 1232, 7556: 1240, 7667: 1256, 7778: 1232, 7889: 1232, 8000: 1232, 8111: 1240, 8222: 1240, 8333: 1224, 8444: 1216, 8556: 1224, 8667: 1232, 8778: 1240, 8889: 1232, 9000: 1216, 9111: 1232, 9222: 1224, 9333: 1248, 9444: 1224, 9556: 1208, 9667: 1224, 9778: 1224, 9889: 1216, 10000: 1192, 10111: 1208, 10222: 1200, 10333: 1192, 10444: 1184, 10556: 1176, 10667: 1176, 10778: 1248, 10889: 1432, 11000: 3472}


prev_meas_100_Hz_up_man = {0: 4855, 111: 4947, 222: 4947, 333: 5042, 444: 5042, 556: 5042, 667: 5042, 778: 5042, 889: 5042, 1000: 5141, 1111: 5141, 1222: 5141, 1333: 5244, 1444: 5141, 1556: 5141, 1667: 5244, 1778: 5244, 1889: 5244, 2000: 5244, 2111: 5351, 2222: 5351, 2333: 5351, 2444: 5351, 2556: 5351, 2667: 5462, 2778: 5462, 2889: 5462, 3000: 5462, 3111: 5462, 3222: 5578, 3333: 5578, 3444: 5578, 3556: 5578, 3667: 5578, 3778: 5700, 3889: 5700, 4000: 5700, 4111: 5700, 4222: 5700, 4333: 5700, 4444: 5826, 4556: 5826, 4667: 5959, 4778: 5959, 4889: 5959, 5000: 5959, 5111: 5959, 5222: 5959, 5333: 5959, 5444: 5959, 5556: 5959, 5667: 5959, 5778: 5959, 5889: 5959, 6000: 5959, 6111: 6097, 6222: 6097, 6333: 6097, 6444: 6097, 6556: 6097, 6667: 6097, 6778: 6097, 6889: 6097, 7000: 6242, 7111: 6242, 7222: 6242, 7333: 6242, 7444: 6242, 7556: 6242, 7667: 6242, 7778: 6242, 7889: 6395, 8000: 6395, 8111: 6395, 8222: 6395, 8333: 6555, 8444: 6555, 8556: 6555, 8667: 6555, 8778: 6555, 8889: 6555, 9000: 6723, 9111: 6723, 9222: 6723, 9333: 6723, 9444: 6900, 9556: 6900, 9667: 6900, 9778: 6723, 9889: 6723, 10000: 6900, 10111: 6900, 10222: 6900, 10333: 7086, 10444: 7086, 10556: 7086, 10667: 7086, 10778: 7086, 10889: 7283, 11000: 7945}
prev_meas_500_Hz_up_man = {0: 5042, 111: 5042, 222: 5141, 333: 5141, 444: 5244, 556: 5244, 667: 5351, 778: 5351, 889: 5351, 1000: 5351, 1111: 5351, 1222: 5462, 1333: 5462, 1444: 5462, 1556: 5462, 1667: 5462, 1778: 5462, 1889: 5462, 2000: 5462, 2111: 5578, 2222: 5578, 2333: 5578, 2444: 5578, 2556: 5700, 2667: 5700, 2778: 5700, 2889: 5700, 3000: 5700, 3111: 5700, 3222: 5826, 3333: 5826, 3444: 5826, 3556: 5826, 3667: 5959, 3778: 5959, 3889: 5959, 4000: 5959, 4111: 5959, 4222: 5959, 4333: 5959, 4444: 6097, 4556: 6242, 4667: 6242, 4778: 6242, 4889: 6242, 5000: 6097, 5111: 6242, 5222: 6242, 5333: 6242, 5444: 6242, 5556: 6242, 5667: 6242, 5778: 6242, 5889: 6395, 6000: 6395, 6111: 6395, 6222: 6555, 6333: 6395, 6444: 6555, 6556: 6395, 6667: 6395, 6778: 6395, 6889: 6555, 7000: 6555, 7111: 6555, 7222: 6555, 7333: 6555, 7444: 6555, 7556: 6723, 7667: 6723, 7778: 6723, 7889: 6900, 8000: 6723, 8111: 6900, 8222: 6900, 8333: 6900, 8444: 6900, 8556: 6900, 8667: 6900, 8778: 7086, 8889: 7086, 9000: 7086, 9111: 7086, 9222: 6900, 9333: 7086, 9444: 7283, 9556: 7283, 9667: 7283, 9778: 7283, 9889: 7283, 10000: 7283, 10111: 7283, 10222: 7491, 10333: 7491, 10444: 7491, 10556: 7491, 10667: 7491, 10778: 7491, 10889: 7711, 11000: 8458}


ENC_RES = 50 # in nm
JOG_STEPS_N = 32
POS_STEP_N = 100
FILE_PATH = 'data.json'
STEP_RATES = [900, 800, 700, 600, 500, 400, 300, 200, 100, 50, 20, 10, 5, 1]


def meas_spc():
    spc = mot.SendRequest('XY25=1')
    time.sleep(2)

def meas_spc_man():
    enc0 = mot.enc_pos
    # print('Jogging forward with JOG_STEPS_N: ', JOG_STEPS_N)
    spc = mot.SendRequest(f'XJ{JOG_STEPS_N}')
    time.sleep(2)
    while mot.State() == tango.DevState.MOVING:
            print('Still jog moving..')
            time.sleep(2)
    enc_diff = abs(mot.enc_pos - enc0)
    # print(f'enc_diff is: {enc_diff}, which equals to {enc_diff * 50} nm')

    spc = mot.SendRequest(f'XJ=-{JOG_STEPS_N}')
    # print('Jogging backward with JOG_STEPS_N: ', JOG_STEPS_N)
    time.sleep(2)
    while mot.State() == tango.DevState.MOVING:
            print('Still jog moving..')
            time.sleep(2)
    X = round(enc_diff / JOG_STEPS_N)
    print(f'Number of encoder steps per jog step is: {X}, which equals to {X * 50} nm jog step')
    # To keep a certain speed e.g. 1 mm/s one has to adjust the wfm-step rate as:
    print(f'Rate for 1 mm/s is : {round(1000000 / (X * ENC_RES))}')
    return round (65546 * 4 / X)

def get_spc():
    spc = mot.SendRequest('XY11')
    return int(spc.split(':')[1])

def write_data(data):
    if os.path.exists(FILE_PATH) and os.path.getsize(FILE_PATH) > 0:
        with open(FILE_PATH, 'rb+') as json_file:
            json_file.seek(-1, os.SEEK_END)
            last_char = json_file.read(1)

            if last_char == b']':
                json_file.seek(-1, os.SEEK_END)
                json_file.truncate()
                json_file.write(b',\n')
    else:
        with open(FILE_PATH, 'w') as json_file:
            json_file.write('[\n')

    with open(FILE_PATH, 'a') as json_file:
        json.dump(data, json_file, indent=4)
        json_file.write('\n]')

pos_f = np.linspace(0, 11000, POS_STEP_N)
pos = np.round(pos_f).astype(int)

spc = dict()

for RATE in STEP_RATES:
    mot.step_rate = RATE
    spc['step_rate'] = int(RATE)
    print('StepRate is set to: ', RATE)
    time.sleep(2)
    for p in pos:
        mot.position = p
        print('Moving to position: ', p)
        time.sleep(2)
        
        while mot.State() == tango.DevState.MOVING:
            print('Still moving..')
            time.sleep(2)
        '''
        print('Measure SPC @pos: ', p)
        meas_spc()
        while mot.State() == tango.DevState.MOVING:
            print('Still moving..')
            time.sleep(2)
        
        s = get_spc()
        '''
        s = meas_spc_man()
        print('Currnet SPC is: ', s)
        spc[int(p)] = s

    print('measured spc are: ', spc)
    curr_meas = spc
    write_data(curr_meas)

# curr_meas = prev_meas_500_Hz_up_man
prev_meas = prev_meas_500_Hz_up_man


x_curr = list(curr_meas.keys())
y_curr = list(curr_meas.values())

x_prev = list(prev_meas.keys())
y_prev = list(prev_meas.values())

plt.plot(x_curr, y_curr, label='Curr meas', color='blue')
plt.plot(x_prev, y_prev, label='Prev meas', color='red')

plt.title('SPC Plot')
plt.xlabel('Pos, micron')
plt.ylabel('SPC Value')
plt.show()
        
