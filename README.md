# audio_stream_arecord_hack

Just a simple package/script to get audio using `arecord` and publishing it in a `audio_common_msgs/AudioData` message.

# Usage

    rosrun audio_stream_arecord_hack audio_streamer_arecord.py

or

    roslaunch audio_stream_arecord_hack audio_stream.launch

You should see your data spitting out at:

    rostopic echo /audio_stream/audio_data

# Custom config

You can change the blocksize, so the amount of data on every message. You can also change the `arecord` command to your needed flags.

Easiest way is to change the launchfile that looks like:

```xml
<launch>
    <node name="audio_stream" 
          pkg="audio_stream_arecord_hack"
          type="audio_streamer_arecord.py" 
          output="screen"
          args="2048 arecord -f S16_LE -r 16000 -t raw">
          <!-- first argument is blocksize, the rest is the full arecord command -->
</launch>
```

# Hacky slow terminal plotting of audio data
Just in case you need a super basic way of plotting the audio data... you can use `gnuplot_audio_data.py` script. It calls with subprocess `gnuplot`.

You may want to tune up the line ` gnuplot.stdin.write("set term dumb 200 60\n")` which sets the number of characters on X and Y to use for the plot.

Looks like this:
```
  40000 ++---------+-----------+----------+----------+-----------+---------++
        +          +           +          +          +         Line1 **A*** +
  30000 ++ A                           AA   A  A    A AA     AA A          ++
        |  A                           *A   AAAA    A AAAA   AAAA           |
        |  A      A      A             AA   AA**    A A*AA   *AAA           |
  20000 ++ A      A      A      A      A*   *A*A   AA*AAAA A AAAA A        ++
        | AA     AA  A A A      A      AA   AAA* A A A*AAA AA **A A         |
  10000 ++AA     AA AA A A*     AAA  A *A   *A*A A A A AAA AA AA* A        ++
        | AA     AA AA A AA     AAAA A AA   AAA* A A A AAAA A AAA A         |
        | * AAA  * AAA *AAA A   A*AA A *AAA AA*A A A A A*AA A *AAA          |
      0 A+A AAA  A **A AAAA A A AA** A AAAA AAA *AAA A AAAA A AA A         ++
        A A *AA  A AAA AA AAA AAAAAAA AAAAA AA* AAAA A AA A A *A A          |
 -10000 A+A AAAA A *AAAAA AAAAAA *AAA AAAAAAA*A A AA   AA A A AA A         ++
        AAA *AAA A A* AA  AAAAAA AAAA AA AAAAAA A A    A  A   A*            |
        |AA AAAAAA AA AA  AA AAA AA*A AA A A AA A A           *A            |
 -20000 ++A AAA AA AA AA  AA     AAAA A* A   *A A A           AA           ++
        |   A   AA AA AA  AA     AAAA AA     AA                             |
 -30000 ++  A   AA AA A   AA     AAAA A      *A                            ++
        |   A   A  AA A   AA     AA   A      AA                             |
        +          +           +          +          +           +          +
 -40000 ++---------+-----------+----------+----------+-----------+---------++
        0         200         400        600        800         1000       1200
```