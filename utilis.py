from pyproj import Transformer
import numpy as np
import copy



def len3d(vec3):
    return np.sqrt(vec3[0]**2+vec3[1]**2+vec3[2]**2)
def subList(list1,list2):
    result=[]
    for i in range(len(list1)):
        result.append(list1[i]-list2[i])
    return result

class LocationConverter:
    def __init__(self) -> None:
        self.trans_GPS_to_XYZ = Transformer.from_crs(4979, 4978)
    def relativeLoc(self,loc1,loc2):
    # lon,lat,alt -> x,y,z
        first=self.trans_GPS_to_XYZ.transform(*loc1)
        rad1=len3d(first)
    
        second=self.trans_GPS_to_XYZ.transform(*loc2)
        rad2=len3d(second)
        z=rad2-rad1
        locx=copy.copy(loc1)
        locy=copy.copy(loc1)
        locx[1]=loc2[1]
        locy[0]=loc2[0]
        distx=subList(self.trans_GPS_to_XYZ.transform(*locx),first)
        disty=subList(self.trans_GPS_to_XYZ.transform(*locy),first)
        distx=len3d(distx)*np.sign(loc2[1]-loc1[1])
        disty=len3d(disty)*np.sign(loc2[0]-loc1[0])
        return [distx,disty,z]
    