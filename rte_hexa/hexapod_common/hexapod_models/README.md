# hexapod_models

Here we keep copies of the different models (URDF, etc) of our hexapod.

## Available models

### URDF

URDF model for Pexod.

> **Notice:** This model has been generated from Xacro files stored in [hexapod_ros]/hexapod_description. It might happen that, by mistake, this URDF model is not in sync with the hexapod_description. If this is the case, you can open an issue here.

[hexapod_ros]: https://github.com/resibots/hexapod_ros

## How to Install

- cd to `hexapod_models` folder
- Configure with `./waf configure --prefix=path_to_install`
- Install with `./waf install`

## How to use it in other projects

The models are installed in `$install_prefix/share/hexapod_models` folder.


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
