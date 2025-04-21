# V2V Communication Project

This project focuses on Vehicle-to-Vehicle (V2V) communication, exploring the theoretical and practical aspects of communication channels in vehicular networks. The aim is to analyze how the environment behaves and to build a channel model in the test conditions.

![Project Overview](./report/pic/readme.png)

## Repository organization

- `./report/` contains the documentation with the `.tex` source code, the pictures and the last compiled pdf
- `./physicalFormulas/`, `./src/` and `./config.m` are the matlab functions used in the main scripts
- `./steps_specific/` and `./main.m` are the scripts to run for the simulation

## Questions

- What is the difference between $$h(\tau)$$ and $$h_{\text{NB}}(\tau)$$?
- How is it possible for $$y(t)$$ to only depend on $$x(t)$$ and not on previous time instances in the narrowband model on slide 1.38?
- Why are the $$\Gamma_{\perp}$$ values real and not complex - *question 3.2 in* `.\physicalFormulas\reflexion.m`
- $$P_{RX}$$ as function of $$P_{TX}$$ seems strange. More likely to be as function of distance - *question 3.3 in* `.\steps_specific\step3_3.m`. Also, how to compute $$P_{RX}$$? $$\frac{V_{OC}^2}{2 Z_a}$$ or $$\frac{V_{OC}^2}{8 Z_a}$$
- *question 3.3*, switch to log-log plot?
- The plot of $$K(d)$$ seems a bit strange, was it really what was asked - *question 3.4 in* `.\steps_specific\step3_4.m`
- *question 3.7*, how to relate the fade margin to the cell range?
- What is the influence on speed. Is it simply a shift in $$f_c$$ depending on the angle of the ray?
- As the maximal frequency of $$H(f)$$ is $$100$$ MHz, is the channel selective to a passband or a baseband frequency range (*step 5*)?
- How to interpret the fact that the 
