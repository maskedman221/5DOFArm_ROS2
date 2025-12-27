import json
import time

class jsonFormat :

    def format_json(self , msg):
        positions = msg.position
        jointjson = {
            'type': 'joints',  # Message type identifier
            'timestamp': time.time(),
            'joints': {
                'j1': float(positions[0]),
                'j2': float(positions[1]),
                'j3': float(positions[2]),
                'j4': float(positions[3]),
                'j5': float(positions[4])
            },
        }
        return jointjson
    
    