<ol>
<li class="has-line-data" data-line-start="1" data-line-end="6">
<p class="has-line-data" data-line-start="1" data-line-end="2">Setting the DDS implementation:</p>
<pre><code class="has-line-data" data-line-start="3" data-line-end="5" class="language-bash"><span class="hljs-built_in">export</span> RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
</code></pre>
</li>
<li class="has-line-data" data-line-start="6" data-line-end="21">
<p class="has-line-data" data-line-start="6" data-line-end="7">ROS 2 command-line tools:</p>
<pre><code class="has-line-data" data-line-start="8" data-line-end="20" class="language-bash">ros2 node list
ros2 node info
ros2 topic list
ros2 topic <span class="hljs-built_in">echo</span>
ros2 topic hz
ros2 service list
ros2 service call
ros2 param list
ros2 param get
ros2 param <span class="hljs-built_in">set</span>
ros2 launch
</code></pre>
</li>
<li class="has-line-data" data-line-start="21" data-line-end="27">
<p class="has-line-data" data-line-start="21" data-line-end="22">Building packages with colcon:</p>
<pre><code class="has-line-data" data-line-start="23" data-line-end="26" class="language-bash">colcon build --packages-select my_package
colcon build --parallel-workers <span class="hljs-number">4</span>
</code></pre>
</li>
</ol>
<p class="has-line-data" data-line-start="27" data-line-end="28">These commands are used for various tasks such as setting environment variables, listing and interacting with nodes and topics, managing parameters, launching ROS 2 systems, and building packages.</p>