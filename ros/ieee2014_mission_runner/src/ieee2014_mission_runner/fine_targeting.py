from __future__ import division

import math
import operator
import sys
import time
import os

import numpy
import cv2

import roslib
from tf import transformations

from ieee2014_mission_runner import render

start_time = int(time.time())
print '/tmp/%i' % (start_time,)
try:
    os.mkdir('/tmp/%i' % (start_time,))
except:
    pass

def product(x):
    return reduce(operator.mul, x)

def normalize(x):
    return (255/(numpy.max(x) - numpy.min(x))*(x - numpy.min(x)) + 0.5).astype(numpy.uint8)

def is_opaque(pixel):
    return pixel[3] >= 128

def pad(x):
    #return cv2.copyMakeBorder(x,
    #    0, x.shape[0],
    #    0, x.shape[1],
    #    cv2.BORDER_CONSTANT, value=(0, 0, 0))
    res = numpy.zeros((2*x.shape[0], 2*x.shape[1]))
    res[:x.shape[0], :x.shape[1]] = x
    return res

def roll_up_left_a_quarter(x):
    return numpy.roll(numpy.roll(x, -x.shape[1]//4, 1), -x.shape[0]//4, 0)

def _cross_correlate(signal, template):
    return numpy.fft.ifft2(
        numpy.fft.fft2(signal) * numpy.fft.fft2(template).conj()
    )[:signal.shape[0]//2, :signal.shape[1]//2].real

def cross_correlate(signal, template, template_weight):
    # at every possible alignment of signal and template:
    # correlation = sum(signal*template*template_weight)
    
    signal_padded = pad(signal - numpy.mean(signal))
    template_padded = roll_up_left_a_quarter(pad(template - numpy.mean(template_weight*template)))
    template_weight_padded = roll_up_left_a_quarter(pad(template_weight))
    
    cross_correlation = _cross_correlate(signal_padded, template_weight_padded*template_padded)
    
    #cv2.imshow('templ', normalize(template_weight))
    #cv2.imshow("a", normalize(cross_correlation))
    #cv2.imshow("b", normalize(_cross_correlate(signal_padded**2, template_weight_padded)))
    #cv2.waitKey()
    
    tmp = cross_correlation / numpy.sqrt(_cross_correlate(signal_padded**2, template_weight_padded))
    res = tmp / math.sqrt(numpy.sum(template_weight_padded*template_padded**2))
    
    print numpy.min(res), numpy.max(res)
    
    return res
    

class Template(object):
    def __init__(self, template_img_with_alpha):
        # result must be 0 where template is transparent
        # sum of result must be 0
        # scale of result doesn't matter
        # scale and bias of input shouldn't matter
        
        assert template_img_with_alpha.shape[2] == 4
        self._orig = template_img_with_alpha.astype(float)
        
        opaque = template_img_with_alpha[:, :, 3] >= 128
        opaque3 = numpy.dstack([opaque]*3)
        
        #opaque_pixels = [pixel[:3].astype(int)
        #    for row in template_img_with_alpha
        #    for pixel in row
        #    if is_opaque(pixel)]
        mean = numpy.sum(numpy.choose(opaque3, [0, template_img_with_alpha[:, :, :3]]), axis=(0, 1))/numpy.sum(opaque)
        #print mean
        
        res = numpy.choose(opaque3, [0, template_img_with_alpha[:, :, :3] - mean])
        
        #res = numpy.array([
        #    [pixel[:3] - mean if is_opaque(pixel) else [0, 0, 0] for pixel in row]
        #for row in template_img_with_alpha])
        
        #cv2.imshow('normalize(res)', normalize(res))
        
        self._template = res # floating point 3-channel image
    
    def match(self, img, debug_images=False):
        img = img.astype(float)
        assert img.shape == (self._orig.shape[0], self._orig.shape[1], 3)
        matchness = product(numpy.maximum(0, cross_correlate(img[:,:,c], self._orig[:,:,c], self._orig[:,:,3]/255.)) for c in xrange(img.shape[2]))
        
        
        important = matchness[matchness.shape[0]//4:matchness.shape[0]*3//4, matchness.shape[1]//4:matchness.shape[1]*3//4]
        
        important_pos = numpy.unravel_index(numpy.argmax(important), important.shape)
        pos = important_pos[0] + matchness.shape[0]//4, important_pos[1] + matchness.shape[1]//4
        
        if True:
            t = int(time.time())
            moved_template = numpy.roll(numpy.roll(self._orig, pos[0]-self._orig.shape[0]//2, 0), pos[1]-self._orig.shape[1]//2, 1)
            #cv2.imshow('moved_template', moved_template)
            debug_img = img.copy()
            debug_img[pos[0]-3:pos[0]+3, pos[1]-3:pos[1]+3] = 0, 0, 0
            debug_img = debug_img//2 + moved_template[:,:,:3]//2
            cv2.imwrite('/tmp/%i/%i-src.png' % (start_time, t), img)
            cv2.imwrite('/tmp/%i/%i-debug.png' % (start_time, t), debug_img)
        
        if debug_images:
            cv2.imshow('/tmp/%i/%i-matchness.png' % (start_time, t), normalize(matchness))
            cv2.imshow('/tmp/%i/%i-src.png' % (start_time, t), img.astype(numpy.uint8))
            cv2.imshow('/tmp/%i/%i-debug.png' % (start_time, t), debug_img.astype(numpy.uint8))
            cv2.waitKey()
        
        '''for y, row in enumerate(template_src):
            for x, pixel in enumerate(row):
                if is_opaque(pixel):
                    img_padded[
                        pos[0]+y-template.shape[0]//2,
                        pos[1]+x-template.shape[1]//2,
                    ] = pixel[:3]
        #img_padded[pos[0]-5:pos[0]+5,pos[1]-5:pos[1]+5] = (0, 0, 0)
        cv2.imshow("xy_normalized", normalize(xy))
        cv2.imshow("img_padded", img_padded)'''
        
        return pos[1] - matchness.shape[1]//2, pos[0] - matchness.shape[0]//2

INCH = 0.0254

target_center = numpy.array([
    -97*INCH / 2 + 1/4*INCH / 2,
    0,
    24*INCH + 5*INCH / 2,
])

def draw_target(r):
    r = r.prepend_transform(transformations.translation_matrix(target_center))
    
    square_side_length = 8*INCH
    r.draw_polygon([
        (0, +square_side_length/2, +square_side_length/2),
        (0, -square_side_length/2, +square_side_length/2),
        (0, -square_side_length/2, -square_side_length/2),
        (0, +square_side_length/2, -square_side_length/2),
    ], (0, 0, 255, 255))
    
    line_width = 3/4*INCH
    r.draw_polygon([
        (0, +square_side_length/2, +line_width/2),
        (0, -square_side_length/2, +line_width/2),
        (0, -square_side_length/2, -line_width/2),
        (0, +square_side_length/2, -line_width/2),
    ], (255, 255, 255, 255))
    r.draw_polygon([
        (0, +line_width/2, +square_side_length/2),
        (0, -line_width/2, +square_side_length/2),
        (0, -line_width/2, -square_side_length/2),
        (0, +line_width/2, -square_side_length/2),
    ], (255, 255, 255, 255))
    
    cutout_radius = 5*INCH / 2
    r.draw_polygon(
        [(0, cutout_radius*math.cos(angle), cutout_radius*math.sin(angle))
            for angle in numpy.linspace(0, 2*math.pi, 20, endpoint=False)],
        (0, 0, 0, 0),
    )
    
    base_width = (3+1/3)*INCH
    base_height = 5*INCH # arbitrary
    r.draw_polygon([
        (0, +base_width/2, -square_side_length/2),
        (0, -base_width/2, -square_side_length/2),
        (0, -base_width/2, -square_side_length/2-base_height),
        (0, +base_width/2, -square_side_length/2-base_height),
    ], (0, 0, 0, 255))

camera_height = 0.15334 + 0.08067

def get(img, pos, P, debug_images=False):
    pos = numpy.array([pos[0], pos[1], camera_height])
    
    dest = render.make_dest(width=img.shape[1], height=img.shape[0])
    r = render.Renderer(dest, P)
    r = r.prepend_transform(render.look_at_mat(target_center + [0, 0, -0.05] - pos)) # aim low to account for offset
    r = r.prepend_transform(transformations.translation_matrix(-pos))
    
    draw_target(r)
    
    template = Template(dest)
    
    if debug_images:
        cv2.imshow('dest', dest)
    
    return template.match(img, debug_images)

if __name__ == '__main__':
    img = cv2.imread(sys.argv[1])
    pos = +0.32, +0.20
    P = numpy.array([
        [462.292755126953, 0, 314.173743238696, 0],
        [0, 466.827423095703, 184.381978537142, 0],
        [0, 0, 1, 0],
    ])
    
    print get(img, pos, P, debug_images=True)
