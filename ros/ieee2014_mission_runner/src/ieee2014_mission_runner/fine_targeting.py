from __future__ import division

import math
import operator
import sys

import numpy
import cv2

import roslib
from tf import transformations

from ieee2014_mission_runner import render

def product(x):
    return reduce(operator.mul, x)

def normalize(x):
    return (x - numpy.min(x))/(numpy.max(x) - numpy.min(x))

def is_opaque(pixel):
    return pixel[3] >= 128

def pad(x):
    return cv2.copyMakeBorder(x,
        0, x.shape[0],
        0, x.shape[1],
        cv2.BORDER_CONSTANT, value=(0, 0, 0))

def cross_correlate(signal, template):
    template_padded = pad(template)
    template_padded_rolled = numpy.roll(numpy.roll(template_padded, -template_padded.shape[1]//4, 1), -template_padded.shape[0]//4, 0)
    cv2.imshow('template_padded_rolled', template_padded_rolled)
    return numpy.fft.ifft2(
        numpy.fft.fft2(pad(signal)) * numpy.fft.fft2(template_padded_rolled).conj()
    )[:signal.shape[0], :signal.shape[1]].real

class Template(object):
    def __init__(self, template_img_with_alpha):
        # result must be 0 where template is transparent
        # sum of result must be 0
        # scale of result doesn't matter
        # scale and bias of input shouldn't matter
        
        self._orig = template_img_with_alpha
        
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
        
        cv2.imshow('normalize(res)', normalize(res))
        
        self._template = res # floating point 3-channel image
    
    def match(self, img):
        assert img.shape == self._template.shape
        matchness = product(numpy.maximum(0, cross_correlate(img[:,:,c], self._template[:,:,c])) for c in xrange(img.shape[2]))
        
        cv2.imshow('normalize(matchness)', normalize(matchness))
        
        important = matchness[matchness.shape[0]//4:matchness.shape[0]*3//4, matchness.shape[1]//4:matchness.shape[1]*3//4]
        
        important_pos = numpy.unravel_index(numpy.argmax(important), important.shape)
        pos = important_pos[0] + matchness.shape[0]//4, important_pos[1] + matchness.shape[1]//4
        
        if True:
            moved_template = numpy.roll(numpy.roll(self._orig, pos[0]-self._orig.shape[0]//2, 0), pos[1]-self._orig.shape[1]//2, 1)
            cv2.imshow('moved_template', moved_template)
            debug_img = img.copy()
            debug_img[pos[0]-3:pos[0]+3, pos[1]-3:pos[1]+3] = 0, 0, 0
            debug_img = debug_img//2 + moved_template[:,:,:3]//2
            cv2.imshow('debug_img', debug_img)
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

def draw_target(r):
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

target_center = numpy.array([
    -97*INCH / 2 + 1/4*INCH / 2,
    0,
    24*INCH + 5*INCH / 2,
])

if __name__ == '__main__':
    P = numpy.array([
        [462.292755126953, 0, 314.173743238696, 0],
        [0, 466.827423095703, 184.381978537142, 0],
        [0, 0, 1, 0],
    ])

    dest = render.make_dest(width=640, height=360)
    r = render.Renderer(dest, P)
    r = r.prepend_transform(render.look_at_mat(target_center))

    draw_target(r.prepend_transform(transformations.translation_matrix(target_center)))
    
    template = Template(dest)
    
    cv2.imshow('dest', dest)
    
    img = cv2.imread(sys.argv[1])
    
    print template.match(img)
