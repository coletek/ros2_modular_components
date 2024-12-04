import sys
import time
import numpy as np

class Sounds():

    sample_rate = 44100  # in Hz 

    sounds = { 'beep': [{'freq': 1000, 'duration': 0.5, 'volume': 0.5}],
               "haptic_feedback":
               [ {'freq': 1200, 'duration': 0.3, 'volume': 0.3}],
               "haptic_feedback2": [{'freq': 2500, 'duration': 0.28, 'volume': 0.3}],
               "haptic_feedback3": [{'freq': 2500, 'duration': 0.28, 'volume': 0.3}],
               "haptic_feedback4":
               [ {'freq': 1500, 'duration': 0.05, 'volume': 0.2},
                 {'freq': 2000, 'duration': 0.05, 'volume': 0.3},
                 {'freq': 2500, 'duration': 0.05, 'volume': 0.4},
                 {'freq': 3000, 'duration': 0.05, 'volume': 0.5},
                 {'freq': 3500, 'duration': 0.05, 'volume': 0.6},
                 {'freq': 4000, 'duration': 0.05, 'volume': 0.7},
                 {'freq': 4500, 'duration': 0.05, 'volume': 0.8},
                 {'freq': 5000, 'duration': 0.05, 'volume': 0.9},
                 {'freq': 5500, 'duration': 0.05, 'volume': 1.0},
                 {'freq': 6000, 'duration': 0.05, 'volume': 1.0} ],
               "haptic_feedback5":
               [ {'freq': 1000, 'duration': 0.02, 'volume': 0.2},
                 {'freq': 1500, 'duration': 0.02, 'volume': 0.3},
                 {'freq': 2000, 'duration': 0.02, 'volume': 0.4},
                 {'freq': 2500, 'duration': 0.02, 'volume': 0.5},
                 {'freq': 3000, 'duration': 0.02, 'volume': 0.6},
                 {'freq': 3500, 'duration': 0.02, 'volume': 0.7},
                 {'freq': 4000, 'duration': 0.02, 'volume': 0.8},
                 {'freq': 4500, 'duration': 0.02, 'volume': 0.9},
                 {'freq': 5000, 'duration': 0.02, 'volume': 1.0},
                 {'freq': 5500, 'duration': 0.02, 'volume': 1.0} ],
               'chromatic_scale':
               [ {'freq': 659, 'duration': 0.15, 'volume': 0.5},
                 {'freq': 988, 'duration': 0.15, 'volume': 0.5},
                 {'freq': 1319, 'duration': 0.15, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.15, 'volume': 0.5},
                 {'freq': 2093, 'duration': 0.15, 'volume': 0.5} ],
               "ascending_scale":
               [ {'freq': 440, 'duration': 0.5, 'volume': 0.5},
                 {'freq': 550, 'duration': 0.3, 'volume': 0.7},
                 {'freq': 660, 'duration': 0.2, 'volume': 0.9},
                 {'freq': 770, 'duration': 0.3, 'volume': 0.7},
                 {'freq': 880, 'duration': 0.5, 'volume': 0.5} ],
               "hammer":
               [ {'freq': 100, 'duration': 0.1, 'volume': 0.3},
                 {'freq': 200, 'duration': 0.1, 'volume': 0.4},
                 {'freq': 300, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 400, 'duration': 0.1, 'volume': 0.6},
                 {'freq': 500, 'duration': 0.1, 'volume': 0.7},
                 {'freq': 600, 'duration': 0.1, 'volume': 0.8},
                 {'freq': 700, 'duration': 0.1, 'volume': 0.9},
                 {'freq': 800, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 900, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 1000, 'duration': 0.1, 'volume': 1.0} ],
               "wrench":
               [ {'freq': 200, 'duration': 0.1, 'volume': 0.3},
                 {'freq': 400, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 600, 'duration': 0.1, 'volume': 0.7},
                 {'freq': 800, 'duration': 0.1, 'volume': 0.9},
                 {'freq': 1000, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 1200, 'duration': 0.1, 'volume': 0.9},
                 {'freq': 1400, 'duration': 0.1, 'volume': 0.8},
                 {'freq': 1600, 'duration': 0.1, 'volume': 0.7},
                 {'freq': 1800, 'duration': 0.1, 'volume': 0.6},
                 {'freq': 2000, 'duration': 0.1, 'volume': 0.5} ],
               "screwdriver":
               [ {'freq': 100, 'duration': 0.1, 'volume': 0.4},
                 {'freq': 200, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 300, 'duration': 0.1, 'volume': 0.6},
                 {'freq': 400, 'duration': 0.1, 'volume': 0.7},
                 {'freq': 500, 'duration': 0.1, 'volume': 0.8},
                 {'freq': 600, 'duration': 0.1, 'volume': 0.9},
                 {'freq': 700, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 800, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 900, 'duration': 0.1, 'volume': 1.0},
                 {'freq': 1000, 'duration': 0.1, 'volume': 1.0} ],
               "chime":
               [ {'freq': 700, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 800, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 900, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1000, 'duration': 0.3, 'volume': 0.5},
                 {'freq': 1100, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1200, 'duration': 0.2, 'volume': 0.5}],
               "bell":
               [ {'freq': 300, 'duration': 0.3, 'volume': 1.0},
                 {'freq': 500, 'duration': 0.3, 'volume': 1.0},
                 {'freq': 700, 'duration': 0.3, 'volume': 1.0},
                 {'freq': 900, 'duration': 0.3, 'volume': 1.0},
                 {'freq': 1100, 'duration': 0.3, 'volume': 1.0} ],
               "alert":
               [ {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5} ],
               "notify":
               [ {'freq': 523, 'duration': 0.2, 'volume': 0.3},
                 {'freq': 1047, 'duration': 0.2, 'volume': 0.3},
                 {'freq': 1568, 'duration': 0.2, 'volume': 0.3} ],
               "timeout":
               [ {'freq': 600, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 500, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 400, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 300, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 200, 'duration': 0.1, 'volume': 0.5},
                 {'freq': 100, 'duration': 0.1, 'volume': 0.5} ],
               "timeout2":
               [ {'freq': 880, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.8} ],
               "error":
               [ {'freq': 440, 'duration': 0.5, 'volume': 0.8},
                 {'freq': 352, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 294, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 262, 'duration': 0.5, 'volume': 0.8} ],
               "error2":
               [ {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 660, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 220, 'duration': 0.2, 'volume': 0.5} ],
               "failure":
               [ {'freq': 440, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 294, 'duration': 0.2, 'volume': 0.8},
                 {'freq': 262, 'duration': 0.5, 'volume': 0.8},
                 {'freq': 196, 'duration': 0.5, 'volume': 0.8},
                 {'freq': 131, 'duration': 0.5, 'volume': 0.8} ],
               "failure2":
               [ {'freq': 220, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 660, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1100, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1320, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1320, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5} ],
               "success":
               [ {'freq': 880, 'duration': 0.3, 'volume': 0.5},
                 {'freq': 1320, 'duration': 0.3, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.3, 'volume': 0.5},
                 {'freq': 1320, 'duration': 0.3, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.3, 'volume': 0.5} ],
               "success2":
               [ {'freq': 659, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1319, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 2093, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1319, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 659, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1319, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 2093, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 1319, 'duration': 0.2, 'volume': 0.5},
                 {'freq': 880, 'duration': 0.2, 'volume': 0.5} ],
                "enable":
                [ {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                  {'freq': 1100, 'duration': 0.2, 'volume': 0.5},
                  {'freq': 1320, 'duration': 0.2, 'volume': 0.5},
                  {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                  {'freq': 2200, 'duration': 0.2, 'volume': 0.5} ],
                "disable": [
                    {'freq': 2200, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1760, 'duration': 0.2, 'volume': 0.2},
                    {'freq': 1320, 'duration': 0.2, 'volume': 0.2},
                    {'freq': 1100, 'duration': 0.2, 'volume': 0.2},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.2} ],
                "reset": [
                    {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.5} ],
                "abort": [
                    {'freq': 1500, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1000, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 500, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 250, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 125, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 62, 'duration': 0.2, 'volume': 0.5} ],
                "starting": [
                    {'freq': 440, 'duration': 0.3, 'volume': 0.6},
                    {'freq': 660, 'duration': 0.3, 'volume': 0.6},
                    {'freq': 880, 'duration': 0.3, 'volume': 0.6},
                    {'freq': 1320, 'duration': 0.3, 'volume': 0.6},
                    {'freq': 1760, 'duration': 0.3, 'volume': 0.6} ],
                "emergency_stop": [
                    {'freq': 440, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.5, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.5, 'volume': 0.5} ],
                "manual_override": [
                    {'freq': 440, 'duration': 0.5, 'volume': 0.7},
                    {'freq': 220, 'duration': 0.4, 'volume': 0.7},
                    {'freq': 110, 'duration': 0.4, 'volume': 0.7} ],                
                "manual_override2": [
                    {'freq': 400, 'duration': 0.1, 'volume': 0.7},
                    {'freq': 500, 'duration': 0.2, 'volume': 0.7},
                    {'freq': 600, 'duration': 0.3, 'volume': 0.7},
                    {'freq': 800, 'duration': 0.5, 'volume': 0.7} ],
                "manual_override3": [
                    {'freq': 659, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 784, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 988, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 1319, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 1760, 'duration': 0.3, 'volume': 0.3},
                    {'freq': 2093, 'duration': 0.3, 'volume': 0.3} ],
                "manual_override4": [
                    {'freq': 523, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 587, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 659, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 784, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 988, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1175, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1319, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1568, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1760, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 1976, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 2093, 'duration': 0.2, 'volume': 0.5} ],               
                "moving": [
                    {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.2, 'volume': 0.5},
                    {'freq': 880, 'duration': 0.2, 'volume': 0.5} ],
                "rolling": [
                    {'freq': 392, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 523, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 659, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 784, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 659, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 523, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 392, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 262, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 392, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 523, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 659, 'duration': 0.15, 'volume': 0.3},
                    {'freq': 784, 'duration': 0.15, 'volume': 0.3} ],
                "oven_on": [
                    {'freq': 293, 'duration': 0.3, 'volume': 0.3},
                    {'freq': 440, 'duration': 0.3, 'volume': 0.3},
                    {'freq': 587, 'duration': 0.3, 'volume': 0.3},
                    {'freq': 784, 'duration': 0.3, 'volume': 0.3} ],
                "oven_off": [
                    {'freq': 880, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 660, 'duration': 0.3, 'volume': 0.5},
                    {'freq': 440, 'duration': 0.6, 'volume': 0.5} ]
              }
    
    def is_sound_available(self, name):
        if not name in self.sounds:
            return False
        return True
                    
    def get_sound(self, name):
        return self.sounds[name]
        
    def get_waveform(self, name):
        tones = self.sounds[name]
        duration = 0.0
        for tone in tones:
            duration += tone['duration']
        time_array = np.arange(0, duration, 1.0/self.sample_rate)
        
        # Create a numpy array of samples for each tone
        samples = []
        for tone in tones:
            freq = tone['freq']
            tone_duration = tone['duration']
            volume = tone['volume']
            tone_samples = np.sin(2 * np.pi * freq * time_array[:int(tone_duration*self.sample_rate)]) * volume
            samples.append(tone_samples)

            # Concatenate the tones into a single numpy array                                                                       
            combined_tones = np.concatenate(samples)

        return duration, samples, combined_tones

if __name__ == '__main__':
    sound = Sounds()
