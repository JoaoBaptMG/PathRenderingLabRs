# PathRenderingLabRs

This is an (abandoned) attempt to port my [SVG preprocessing and rendering project](https://github.com/JoaoBaptMG/PathRenderingLab) from C# to Rust. It is posted here just for the sake of it, you are free to fork it and contribute to it however you feel like, but contribution of the original author of this repository might not be made anymore.

So far, the Rust port only comprises the generation of primitives from a single path. It has no support for stroke generation, reading from arbitrary SVGs, and not even implements the rendering part of the code (this would require linking to a proper graphics library). The code *is* almost line-to-line similar to its C# counterpart, only substituting explicit references for indices where there were some difficulties regarding the compiler.

## Testing

There are two test files in the repository, `monogame.txt` and `bigpath.txt` (none of their content is my work though). You can input however any text file that has a path description according to the SVG specification.

## Contributing

Feel free to fork this repository and push to it however you feel like. Pull requests will be reviewed and accepted as well.
