# bevy_ballistic

![demo](./demo.gif)

Simple launch velocity calculation for game projectiles to hit a specified target based on given gravity. Only depending on `bevy_math` for vector math.
Based on this [article](https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/) by Forrest Smith

## How to run example

```
cargo r --example bevy
```

## Todo

* support aiming at predicted future position based on moving target velocity
