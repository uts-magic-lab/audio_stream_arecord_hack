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