import os
import time
import glob

class OneWireThermocouples:

    def get_temperature_raw(self, file_path):
        f = open(file_path, 'r')
        lines = f.readlines()
        f.close()
        return lines

    def get_temperature(self, sensor_id):
        file_path = "/sys/bus/w1/devices/%s/w1_slave" % sensor_id
        lines = self.get_temperature_raw(file_path)
        try:
            if lines[0].strip()[-3:] != 'YES':
                return -1
        except:
            return -1
        temp_output = lines[1].find('t=')
        if temp_output != -1:
            temp_string = lines[1].strip()[temp_output+2:]
            temp_c = float(temp_string) / 1000.0                        
            return temp_c
        else:
            return -1
        
    def find_all_sensor_ids(self):
        device_folder = '/sys/bus/w1/devices'
        sensor_ids = []
        for device_name in os.listdir(device_folder):
            if device_name.startswith('28-') or device_name.startswith('3b-'):
                sensor_ids.append(device_name)
        return sensor_ids
        
def main():
    t = OneWireThermocouples()

    # takes about 13sec to read 8x devices
    ids = t.find_all_sensor_ids()
    for i in ids:
        print ("sensor[%s]=%f" % (i, t.get_temperature(i)))
    
    #file_paths = t.file_path_gen(1)
    #i = 0
    #print ("%f " % time.time(), end='')
    #for file_path in file_paths:
    #    print ("%.3f " % (t.read_temp(file_path + '/w1_slave')), end='')
    #    i += 1
    #print ("")

if __name__ == '__main__':
    main()
