from Datagrams import BaseDatagram

class Flightstate_Dgram(BaseDatagram.BaseDatagram):


    def __init__(self):
        super().__init__()
        self.byteorder = ">"

        self.add_entry("llh_lat", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("llh_lon", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("hae", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("agl", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("ned_north", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("ned_east", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("ned_down", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("phi", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("theta", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("psi", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("tas", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("gnd_spd", 0, data_type=BaseDatagram.DataTypes.FLOAT)
        self.add_entry("roc", 0, data_type=BaseDatagram.DataTypes.FLOAT)
