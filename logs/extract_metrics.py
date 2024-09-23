#!/usr/bin/env python3

class MetricsData(object):
    def __init__(self):
        self.data = {}
        self.data['R'] = {}
        self.data['H'] = {}
        
    def load_data(self, file_path):
        try:
            with open(file_path, "r") as file:
                file.readline()
                for line in file:
                    if "R" in line and not "COST" in line and not "H" in line and not "Roxanne" in line:
                        split_data = line.split()
                        time = float(split_data[0])
                        if not time in self.data['R']:
                            self.data['R'][time] = {}
                                                    
                        if "SPEED" in line:
                            self.data['R'][time]['speed'] = float(split_data[3])
                        
                        elif "VEL" in line:
                            self.data['R'][time]['velocity'] = [float(split_data[3]), float(split_data[4]), float(split_data[5])]
                        else:
                            self.data['R'][time]['position'] = [float(split_data[3]), float(split_data[4]), float(split_data[5])]

                    elif "H" in line and not "Roxanne" in line:
                        split_data = line.split()
                        time = float(split_data[0])
                        h_id = split_data[2].split('_')[0]
                        if not h_id in self.data['H']:
                            self.data['H'][h_id] = {}
                        if h_id in self.data['H']:
                            if not time in self.data['H'][h_id]:
                                self.data['H'][h_id][time] = {}
                        
                        if "SPEED" in line:
                            self.data['H'][h_id][time]['speed'] = float(split_data[3])
                            
                        elif "VEL" in line:
                            self.data['H'][h_id][time]['velocity'] = [float(split_data[3]), float(split_data[4]), float(split_data[5])]
                            
                        elif "COST_REACT" in line:
                            self.data['H'][h_id][time]['react'] = float(split_data[3])
                            
                        elif "COST_SHOCK" in line:
                            self.data['H'][h_id][time]['shock'] = float(split_data[3])
                            
                        elif "COST_VISIBILITY" in line:
                            self.data['H'][h_id][time]['visib'] = float(split_data[3])
                            
                        elif "COST_FEAR" in line:
                            self.data['H'][h_id][time]['fear']= float(split_data[3])

                        elif "COST_PANIC" in line:
                            self.data['H'][h_id][time]['panic'] = float(split_data[3])
                            
                        else:
                            self.data['H'][h_id][time]['position'] = [float(split_data[3]), float(split_data[4]), float(split_data[5])]
               
        except Exception as e:
            print(f"Load Data Error: {e}")
    
    def calculate(self, interval, type="time"):
        if type == "time":
            robot_data = [v for k, v in self.data['R'].items() if interval[0] <= k <= interval[1]]
            humans_data = {}
            hids = self.get_human_ids()
            for id in hids:
                humans_data[id] = [v for k, v in self.data['H'][id].items() if interval[0] <= k <= interval[1]]
            # Metrics
            fear = []
            panic = []
            react = []
            shock = []
            visib = []
            for id in hids:
                for human_data in humans_data[id]:
                    fear.append(human_data['fear'])
                    panic.append(human_data['panic'])
                    react.append(human_data['react'])
                    shock.append(human_data['shock'])
                    visib.append(human_data['visib'])
            # print(max(fear),max(panic), max(react))
            print(len(robot_data), len(humans_data[id]))
        
    def get_human_ids(self):
        return list(self.data['H'].keys())