# Qspice digital controller

I plan to organize this better later, but for now the file "digital_controller.cpp" is currently the focus of this repository.

## To-do

1. Reestructure gains as a generic second order transfer function, i.e.

```math 
    C(z) = \frac{K_{n0} + K_{n1}z^{-1} + K_{n2}z^{-2}}{1 + K_{d1}z^{-1} + K_{d2}z^{-2}}
```


- done

2. Adjust the triangular carrier mode

- done

3. Implement sampling time options
    - One in valley (triangular and sawtooth)
    - Another in middle of ascending carrier (triangular and sawtooth)
    - And another in the middle of descending carrier (only triangular)