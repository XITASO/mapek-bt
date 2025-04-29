from Utils import Conversion

import enum


class UniqueCoordinates(enum.Enum):
    BRUNSWICK_AIRPORT = Conversion.LLH_t(Conversion.deg2rad(52.319167), Conversion.deg2rad(10.555278), 90)
    BAD_PYRMONT = Conversion.LLH_t(Conversion.deg2rad(51.984165), Conversion.deg2rad(9.248279), 0)
    ELM_SOUTH_RIM = Conversion.LLH_t(Conversion.deg2rad(52.18777778), Conversion.deg2rad(9.245371), 0)
    THAL_WESERBERGLAND = Conversion.LLH_t(Conversion.deg2rad(51.98805556), Conversion.deg2rad(9.31388889), 0)
    LEIFERDE_PA_FLIGHT_TEST = Conversion.LLH_t(Conversion.deg2rad(52.47750000), Conversion.deg2rad(10.40555556), 0)
