## Reflections

### Components of a PID controller

- **P**roportional: reacts to present errors immediately and (obviously) proportionally 
- **I**ntegral: reacts to the total error seen in the past including the current error. The integral term allows the system to correct for any biases
- **D**erivative: reacts to the trend in the error, acting as a dampening effect on the proportional term. Helps to smooth out the control system.

### Choosing the hyper parameters

I initially wanted to try the Twiddle algorithm to tune the parameters, but unlike in the Python simulation, it's not as easy to rerun this simulation automatically many times.

Instead, I chose to try a binary search on each of the three hyper parameters.

I first set Ki and Kd to 0.0 and tuned Kp. During this phase, I calculated the absolute value of the minimum cross track error, where the slope of the CTE was closest to 0. In an oscillation, this would be where the peak or trough was closest to the target value.

I then performed a binary search to narrow in on an optimal Kd.

Finally, I tuned Ki in a similar manner. This time, though, I calculated an average squared cross track error, by taking the total squared and dividing by the count of the current telemetry packet. I would look at this value at the same point of the track each time to make a fair comparison.

#### Changing the hyper parameters

Oddly enough, even though I chose parameters that were "optimal" according to my average squared CTE, I felt that the car did not drive smoothly. It seemed to jerk around a lot. I *think* this is because it was aggressively correcting for the error, but the parameters intended to smooth it out were not as helpful.

So on a whim, I copied the parameters Sebastian used for the robot in the PID Controller lesson and ran the simulator with those. The car drove *much* more smoothly, despite having a higher overall error rate. I unsystematically fiddled with the numbers a little bit until I was happy with the overall performance.

Even though the error is higher, the car seems to drive better and it never gets close to leaving the lane.

I have left my "optimized" hyper parameters in the code, but commented out, for comparison.
