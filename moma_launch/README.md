# moma_launch

This is a pure __configuration and launch only__ package. 
The launch directory contains a set of launch files used for various control and perception tasks. 

It is often good practice to add each dependent package as a _run dependency_ but we do not do this year.
In fact, one might want this package but not all his run dependencies as a single launch file is needed. 

### Good practice

Keep configuration files semantically ordered in `config/<task-name>` such that it is easy to navigate the package. 
	