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
- The plot of $$K(d)$$ seems a bit strange, was it really what was asked - *question 3.4 in* `.\steps_specific\step3_4.m`
- What is the influence on speed. Is it simply a shift in $$f_c$$ depending on the angle of the ray?
- How to interpret the fact that $$h_{\text{TDL}}(t)$$ is non zero even before the ray has traveled the distance?
- Is $$H_{\text{TDL}}(f)$$ of some interest?
