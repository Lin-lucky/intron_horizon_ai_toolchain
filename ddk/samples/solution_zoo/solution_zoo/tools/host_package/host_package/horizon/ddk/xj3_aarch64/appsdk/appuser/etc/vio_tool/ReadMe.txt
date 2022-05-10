1.SIF testpattern
./vio_tool -v "./cfg/pattern_dual_sif_isp_ipu_offline_1952x1097.json" -r 5 -p 2 -c 1 -t 1 -l 0 -m 1 -f 1 -g 1 

cam cfg:

num     sensor&mode
0      imx327 linear    i2c 5
1      imx327 dol2      i2c 5
2      os8a10 linear    i2c 0
3      os8a10 dol2      i2c 0
4      ar0233 pwl
5      ar0144 pwl

