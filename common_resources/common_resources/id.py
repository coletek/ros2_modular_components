class ID:
    
    def get_machine_id(self):
        try:
            with open('/etc/machine-id', 'r') as file:
                machine_id = file.read().strip()
            return machine_id
        except FileNotFoundError:
            print("Error: /etc/machine-id file not found")
            return None
