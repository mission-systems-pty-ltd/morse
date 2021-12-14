import logging; logger = logging.getLogger("morse." + __name__); logger.setLevel(logging.INFO)
import os; logger.info(os.path.basename(__file__))
import os; print("File: ", os.path.basename(__file__), flush=True)
from morse.core import blenderapi

def change_light_energy():
    co = blenderapi.controller()
    ow = co.owner
    
    ow.energy = ow['Energy'] if ow['On'] else 0
