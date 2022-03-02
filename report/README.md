### README

- Run the script in the `drivers` folder `run.sh` to load the module `cryptocard_mod.ko` into the kernel. This script runs `make` and then loads the module.
- Alternative to running the script:
```
cd drivers
make
sudo insmod cryptocard_mod.ko
```

- Execute `make` as explained in the root `README.md` file and execute any test case.
