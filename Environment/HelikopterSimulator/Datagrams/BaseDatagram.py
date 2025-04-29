from enum import Enum
from collections import OrderedDict
import struct


class DatagramTypes(Enum):
    BASE = "Base"
    PATH_REQUEST = "Path_Request"
    EURONAV_STATE = "Euronav_State"
    EURO_FLIGHTPATH = "Euro_Flightpath"
    EURO_FLIGHTPATH_ANNOTATION = "Euro_Flightpath_Annotation"
    FLIGHTPATH = "Flightpath"
    NO_FLY_ZONE = "No Fly Zone"


class DataTypes(Enum):
    INT = 'i'
    UNSIGNED_INT = 'I'
    DOUBLE = 'd'
    LONG = 'l'
    UNSIGNED_LONG = 'L'
    LONG_LONG = 'q'
    FLOAT = 'f'
    CHAR = 'B'
    SHORT = 'h'
    UNSIGNED_SHORT = 'H'
    ARRAY_DOUBLE = 'd '
    ARRAY_FLOAT = 'f '
    ARRAY_INT = 'i '
    ARRAY_SHORT = 'h '
    # ARRAY_CHAR = 'B '


class BaseDatagram:
    def __init__(self):
        self.entries = OrderedDict()
        self.type = DatagramTypes.BASE
        self.byteorder = "<"

    def __str__(self):
        output = ""

        for _, entry in self.entries.items():
            output += entry.name + " = " + str(entry.data) + "\n"

        return output

    def add_entry(self, name, data, data_type=DataTypes.DOUBLE):
        new_entry = BaseDatagramEntry(name, data, data_type, byteorder=self.byteorder)
        self.entries[name] = new_entry

    def set_data(self, entry_to_set, data):
        """
        This method is intended to be a setter function for data. It makes a sanity check to the corresponding data.
        :param entry_to_set: The representation of the entry to be accessed.
        :param data: The data. Should be parseable to the correct format.
        :return:
        """
        entry = self.entries[entry_to_set]
        data_in_correct_format = entry.convert_to_correct_data_type(data)

        entry.data = data_in_correct_format

    def serialize(self):
        serialized = b""

        for _, entry in self.entries.items():
            serialized += entry.to_bytes()

        return serialized

    def get_hex_string(self) -> str:
        serialized = self.serialize()
        hex_string = serialized.hex()

        result = ""

        for i in range(0, len(hex_string), 2):
            result += "\\x" + hex_string[i:i+2]

        return result

    def deserialize(self, bytes):
        """
        Function to deserialize a given byte stream

        :param bytes:
        :return:
        """

        data_counter = 0

        for key, entry in self.entries.items():
            data_type_length = struct.calcsize(entry.data_type.value)
            # print(key, data_type_length)

            if entry.data_type in (DataTypes.ARRAY_DOUBLE, DataTypes.ARRAY_FLOAT, DataTypes.ARRAY_INT, DataTypes.ARRAY_SHORT):
                """Special case handling for arrays"""
                array_size = entry.get_array_length()
                data_type_length = array_size * data_type_length

            current_byte_stream = bytes[data_counter:data_counter+data_type_length]

            entry.from_bytes(current_byte_stream)

            data_counter += data_type_length
            # print(len(bytes)-data_counter)
            

        return


class BaseDatagramEntry:
    def __init__(self, name, data, data_type = DataTypes.DOUBLE, byteorder="<"):
        self.byteorder = byteorder

        self.name = name
        self.data_type = data_type

        self.data = self.convert_to_correct_data_type(data)

    def get_array_length(self):
        if isinstance(self.data, list):
            return len(self.data)
        else:
            raise TypeError("Stored data is no list!")

    def to_bytes(self):
        # This is a workaround for the problem that float entries matched the same name for FLOAT and ARRAY_FLOAT
        if self.data_type == DataTypes.ARRAY_FLOAT or self.data_type == DataTypes.ARRAY_SHORT or self.data_type == DataTypes.ARRAY_INT or self.data_type == DataTypes.ARRAY_DOUBLE:
            result_string = b""

            for entry in self.data:
                result_string += struct.pack(self.byteorder + self.data_type.value, entry)

            return result_string
        # if self.data_type == DataTypes.ARRAY_CHAR:
            # result_string = b""

            # for entry in self.data:
                # result_string += struct.pack(self.byteorder + self.data_type.value, entry)

            # return result_string
        else:
            return struct.pack(self.byteorder + self.data_type.value, self.data)

    def from_bytes(self, bytestream):
        """
        Conversion back from a given byte stream, returns the length of extracted bytes.

        :param bytestream: The bytes, is expected to be in the corresponding length, as specified in the DataTypes Enum
        :return: length of bytes extracted
        """
        temp = len(bytestream)

        if self.data_type in (DataTypes.ARRAY_INT, DataTypes.ARRAY_FLOAT, DataTypes.ARRAY_SHORT, DataTypes.ARRAY_DOUBLE):
            for i in range(0, len(self.data)):
                data_type_length = struct.calcsize(self.data_type.value)
                current_index = i * data_type_length
                current_bytestream = bytestream[current_index:current_index + data_type_length]

                self.data[i] = struct.unpack(self.byteorder + self.data_type.value, current_bytestream)[0]
        else:
            # print(len(bytestream))
            self.data = struct.unpack(self.byteorder + self.data_type.value, bytestream)[0]

    def convert_to_correct_data_type(self, data):
        """
        Converts the specified data to data of the corresponding type. This is happening to allow the specification with
        format strings in the to_byte and from_bytes method.

        :param data: The data to be converted in an arbitrary, but valid format.
        :return:
        """

        if self.data_type == DataTypes.INT or self.data_type == DataTypes.UNSIGNED_INT \
                or self.data_type == DataTypes.LONG_LONG \
                or self.data_type == DataTypes.SHORT or self.data_type == DataTypes.UNSIGNED_SHORT:
            return int(data)
        elif self.data_type == DataTypes.FLOAT or self.data_type == DataTypes.DOUBLE:
            return float(data)
        elif self.data_type == DataTypes.CHAR:
            return int(data)
        elif self.data_type in (DataTypes.ARRAY_FLOAT, DataTypes.ARRAY_DOUBLE):
            new_data = []
            for entry in data:
                new_data.append(float(entry))

            return new_data
        elif self.data_type in (DataTypes.ARRAY_INT, DataTypes.ARRAY_SHORT):
            new_data = []
            for entry in data:
                new_data.append(int(entry))
          
            return new_data
        # elif self.data_type == DataTypes.ARRAY_CHAR: # weig_rl
            # new_data = []

            # for entry in data:
                # new_data.append(char(entry))

            # return new_data
