from __future__ import division

import sys

import roslib

import numpy
import cv2

def fix(x):
    return (x - numpy.min(x))/(numpy.max(x) - numpy.min(x))

def is_opaque(pixel):
    return pixel[3] >= 128

def templateify(img_with_alpha):
    # result must be 0 where template is transparent
    # sum of result must be 0
    # scale of result doesn't matter
    # scale and bias of input shouldn't matter
    print img_with_alpha.shape
    print img_with_alpha[0, 0]
    
    opaque_pixels = [pixel[:3].astype(int)
        for row in img_with_alpha
        for pixel in row
        if is_opaque(pixel)]
    mean = sum(opaque_pixels)/len(opaque_pixels)
    
    res = numpy.array([
        [pixel[:3] - mean if is_opaque(pixel) else [0, 0, 0] for pixel in row]
    for row in img_with_alpha])
    
    return res # floating point 3-channel image

#img = cv2.imread(sys.argv[1])
#cv2.imshow("img", img)

template_path = roslib.packages.resource_file('ieee2014_mission_runner', 'data', 'template.png')
template_src = cv2.imread(template_path, -1)
#cv2.imshow("template_src", template_src)
template = templateify(template_src)
#cv2.imshow("template_fixed", fix(template))

#print map(cv2.getOptimalDFTSize, img.shape)

def pad(x, resulting_size, borderType=cv2.BORDER_REPLICATE, value=None):
    return cv2.copyMakeBorder(x,
        (resulting_size[0] - x.shape[0])//2, (resulting_size[0] - x.shape[0])//2,
        (resulting_size[1] - x.shape[1])//2, (resulting_size[1] - x.shape[1])//2,
        borderType, value=value)
def set_padding_to(x, start_size, value):
    x = x.copy()
    resulting_size = x.shape
    keep = x[
        (resulting_size[0] - start_size[0])//2:(resulting_size[0] + start_size[0])//2,
        (resulting_size[1] - start_size[1])//2:(resulting_size[1] + start_size[1])//2,
    ].copy()
    x[:,:] = value
    x[
        (resulting_size[0] - start_size[0])//2:(resulting_size[0] + start_size[0])//2,
        (resulting_size[1] - start_size[1])//2:(resulting_size[1] + start_size[1])//2,
    ] = keep
    return x

def get(img):
    size = img.shape[0]*2, img.shape[1]*2
    img_padded = pad(img, size, cv2.BORDER_CONSTANT, (255*2//3, 255//3, 255//3))
    template_padded = pad(template, size, cv2.BORDER_CONSTANT, (0., 0., 0.))
    template_padded = numpy.roll(numpy.roll(template_padded, template_padded.shape[1]//2, 1), template_padded.shape[0]//2, 0)
    
    print img_padded.dtype
    print template_padded.dtype

    #cv2.imshow("img_padded", img_padded)
    #cv2.imshow("template_padded", fix(template_padded))

    def product(x):
        import operator
        return reduce(operator.mul, x)
    xy = product(numpy.maximum(0, numpy.fft.ifft2(
        numpy.fft.fft2(img_padded[:,:,c]) * numpy.fft.fft2(template_padded[:,:,c]).conj()
    ).real) for c in xrange(3))
    
    xy -= numpy.min(xy)
    
    xy = set_padding_to(xy, (img.shape[0]-template.shape[0], img.shape[1]-template.shape[1]), 0)
    
    pos = numpy.unravel_index(numpy.argmax(xy), size)
    print pos
    for y, row in enumerate(template_src):
        for x, pixel in enumerate(row):
            if is_opaque(pixel):
                img_padded[
                    pos[0]+y-template.shape[0]//2,
                    pos[1]+x-template.shape[1]//2,
                ] = pixel[:3]
     #img_padded[pos[0]-5:pos[0]+5,pos[1]-5:pos[1]+5] = (0, 0, 0)
    
    cv2.imshow("xy_fixed", fix(xy))
    cv2.imshow("img_padded", img_padded)
    
    return pos
