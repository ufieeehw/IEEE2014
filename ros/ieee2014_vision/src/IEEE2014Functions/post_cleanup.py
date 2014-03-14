#Post-cleanup
import cv2
import numpy as np
import sys

INCH = 0.0254
course_length = (97 - 3/4 * 2) * INCH
course_width = (49 - 3/4 * 2) * INCH
	
def cvtLogical_to_Forrest((x,y)):
    course_length = (97 - 3/4 * 2) * INCH
    course_width = (49 - 3/4 * 2) * INCH
    return (x-course_length/2,y-course_width/2)

def cvtXLogical_to_Forrest(x):
    
    return x - course_length/2

def get_arc_point(y,radius):
    x = np.sqrt( np.square(radius) - np.square(y) )
    return x
    
def find_best_fit((x,y)):
    
    line_x_coordinates = map(cvtXLogical_to_Forrest,(
        35.25*INCH,
        get_arc_point(y, 49*INCH),
        57.75*INCH,
    ))
    print line_x_coordinates
    diffs = np.abs(np.subtract(x,line_x_coordinates))

    least_diff,idx = min((val,idx) for (idx, val) in enumerate(diffs))
    
    return( (line_x_coordinates[idx], y) )


#Unit test:
if __name__ == '__main__':
    if len(sys.argv) > 1:
	    number = sys.argv[1]
    else:
	    number = 0
    #print get_arc_point(0,49*INCH) - course_length/2

    print find_best_fit((course_length/2, 0.38))
    print find_best_fit((0,0.38))
    print find_best_fit((-1,0.038))
    
    
