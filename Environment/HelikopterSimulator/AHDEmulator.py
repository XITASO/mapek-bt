import sys
sys.path.append("..")
sys.path.append("../..")


from Datagrams.flightstate import Flightstate_Dgram

from Communication.UDPSender import UDPSender
from Communication.UDPReceiver import UDPReceiver

from Utils import Conversion

import json
import time
import math
from threading import Thread
from datetime import datetime


class AHDEmulator:
    def __init__(self, config = "config.json"):
        self.Config =  self._readConfig(config)

        self.State = Flightstate_Dgram()
        self.StateSender = UDPSender(self.State.serialize(),self.Config["interfaces"]["state_sender"]["address"],self.Config["interfaces"]["state_sender"]["port"])
        

        self.ref = Conversion.HomeReference(0,0)
        self.ned = Conversion.NED_t()
        self.llh = Conversion.LLH_t()
        
        self.hasFP = False
        self.orientation = [0,0,0]
        self.v = 0
        self.climbRate = 0


        self.path = []
        
        self._setUpState()
        self._readFlightpath(self.Config["path_file"])

        self.sendingFrequency = 40
        self.Tick = 1/self.sendingFrequency

        print(self.path[-1][0].north, self.path[-1][0].east)


    def _readConfig(self, configFile):
        with open(configFile, 'r') as f:
            data = json.load(f)
        return data

    def _setUpState(self):
        # read start from config
        latRef = Conversion.deg2rad(self.Config["state"]["reference"]["lat"])
        lonRef = Conversion.deg2rad(self.Config["state"]["reference"]["lon"])

        latPos = Conversion.deg2rad(self.Config["state"]["position"]["lat"])
        lonPos = Conversion.deg2rad(self.Config["state"]["position"]["lon"])
        hellPos = self.Config["state"]["position"]["h_ell"]
        print(latRef, lonRef)
        # calc ned + llh pos
        self.ref = Conversion.HomeReference(latRef, lonRef)
        self.llh.latitude = latPos
        self.llh.longitude = lonPos
        self.llh.height = hellPos

        self.ned = self.ref.llh2ned(self.llh)

        self._updatePos()

    def Start(self):
        self.StateSender.init_sending_with_frequency(self.sendingFrequency)

        while True:
            self.FollowPath()
            time.sleep(1)

    def _updatePos(self):

        # update AHD State

        self.State.set_data("llh_lat", Conversion.rad2deg(self.llh.latitude) )
        self.State.set_data("llh_lon", Conversion.rad2deg(self.llh.longitude))

        self.State.set_data("hae", self.llh.height)
        self.State.set_data("agl", self.llh.height-400)

        self.State.set_data("phi", Conversion.rad2deg(self.orientation[0] ))
        self.State.set_data("theta", Conversion.rad2deg(self.orientation[1]) )
        self.State.set_data("psi", Conversion.rad2deg(self.orientation[2]) )

        self.State.set_data("tas", self.v )
        self.State.set_data("gnd_spd", self.v )

        self.State.set_data("roc", self.climbRate )

        self.State.set_data("ned_north",self.ned.north)
        self.State.set_data("ned_east", self.ned.east) 
        self.State.set_data("ned_down", self.ned.down)

        self.StateSender.message = self.State.serialize()

    def tp2path(self):
        self.PathFollowing = False

        if not self.hasFP:
            return
        
        time.sleep(2*self.Tick)       
        self.ned.east = self. path[0][0].east
        self.ned.north = self.path[0][0].north
        self.ned.down = self. path[0][0].down
        self.v = 0
        self.turnRate = 0
        self.climbRate = 0
        self.llh = self.ref.ned2llh(self.ned)
        self._updatePos()

    def FollowPath(self):

        if not self.hasFP:
            return

        self.tp2path()

        for i in range(1,len(self.path)):
            dist = self.path[i-1][0].distance(self.path[i][0])
            
            if dist <= 0:
                continue

            self.v = self.path[i-1][1]
            ds = self.v*self.Tick
            num_steps = int(dist/(5*ds))
            DX = (self.path[i][0].north-self.path[i-1][0].north) / num_steps
            DY = (self.path[i][0].east -self.path[i-1][0].east)  / num_steps
            DH = (self.path[i][0].down -self.path[i-1][0].down)  / num_steps
            DS = math.sqrt(DY**2+DX**2)

            self.orientation[2] = math.atan2(DY,DX)
            self.climbRate = math.atan(-DH/DS)*self.v

            for j in range(num_steps):
                self.ned.east = self. path[i-1][0].east  + j*DY
                self.ned.north = self.path[i-1][0].north + j*DX
                self.ned.down = self. path[i-1][0].down  + j*DH
                self.llh = self.ref.ned2llh(self.ned)
                self._updatePos()
                time.sleep(self.Tick)
    
    def _readFlightpath(self, path_file):
        with open(path_file, 'r') as f:
            path = json.load(f)
            for point in path["points"]:
                fp = (self.ref.llh2ned(
                    Conversion.LLH_t(
                        math.pi/180 * point["lat"],
                        math.pi/180 * point["lon"],
                        point["height"],
                    )),
                    point["v"]
                )
                self.path.append(fp)
            self.hasFP = True