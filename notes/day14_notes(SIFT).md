10.21.2014
comprobo!

image as a function

input: x, y location
output: brightness

orientation histograms:

the different buckets
amt we put in histogram is proportional to the magnitude of the vector

circle can be the weighting function
(care more about the things in the center; design decision, etc)
depends on how global or local you want your descriptor to be!

loss of info from histogram-ing: if you scrambled up the points, the histogram looks the same
so... to deal with this SIFT chops up the image patch into a bunch of discrete cells, and computes a separate orientation histogram from each cell

next slide:
magnitude of the vector represents how much is in the bin

to make it rotation invariant:
find the dominant orientation
rotate the image patch so the max is pointing up


histogram tradeoff: 
translation invariance for loss of specificity of where things happened

playing with the visualize_sift script :D
    more effects in the center than in the corners. effect of weighting function things
    tinker with angle, etc



think about object tracking in human first:
    starting from where you were previously
    worrying about other things
    pattern of motion (has it gotten bigger or smaller?)
    filtering


color histogram thing
the black/white spots of things
estimate of how similar the thing is to the original patch

first click: freeze the frame, click top left corner, click bottom right corner


light does funky things with the color histogram (reflections change colors a lot)
shiny things are difficult

meanshift
algorithm for finding areas of high density
our best guess on where the object is is where there are lots of kp matches (a cluster, so to speak)

what makes a pt high density?
weighting function, based on how close it is to xc

||22 (that notation... squared distance between two vectors)

intuitively: 
the bigger the distance, e to a negative number, get a big number
to smaller the distance, e to the zero-ish, get a 1
gamma determines how steeply the weights "fall off"


if gamma was super big
it would just go to the first guess

if gamme was super small ("diffuse")
everything would have the same weight
and then it'd just go to the unweighted thing



put xc on a place where on average, the weights are the highest

shift the pt to the weighted center

1. calculate weights for initial guess
2. move that pt to the weighted avg of all pts
repeat until satisfied (i.e. within treshold)


- weighted center vs unweighted center
well, weighted center will help you find clusters
if there are multiple clusters, the cluster you converge to somewhat depends on where you start


converted image to a hue image

sometimes hue histograms work better for image things

- CAMshift

so... biggested issue with meanshift is that the window size does not change... so we should use camshift instead
bounding box can also change and stuff

