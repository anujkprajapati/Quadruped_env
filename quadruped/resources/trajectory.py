import math
import numpy as np


class Path:
    def __init__(self):
        pass
    

    def get_location(odd,n_step,count):
        
        stride = 0.05
        height = 0.03
        basex = n_step/count * stride
        # basex = posx + stride
        theta = n_step%count * np.pi
        if odd ==0:
            # zdirection
            h2z = 0.05
            h3z = 0.05
            h1z = 0.05 + height*np.sin(theta)
            h4z = 0.05 + height*np.sin(theta)

            #xdirection
            h1x = basex + 0.102 - stride/2 * np.cos(theta)
            h4x = basex - 0.102 - stride/2 * np.cos(theta)
            h2x = basex + 0.102
            h3x = basex - 0.102

        else: 
            # zdirection
            h1z = 0.05
            h4z = 0.05
            h2z = 0.05 + height*np.sin(theta)
            h3z = 0.05 + height*np.sin(theta)

            #xdirection
            h2x = basex + 0.102 - stride/2 * np.cos(theta)
            h3x = basex - 0.102 - stride/2 * np.cos(theta)
            h1x = basex + 0.102
            h4x = basex - 0.102
        print(f'the count is{count}')
        return h1x, h2x, h3x, h4x, h1z, h2z, h3z, h4z




            

