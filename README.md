![Unity SSR](https://github.com/Xerxes1138/UnitySSR/blob/master/UnitySSRV2.png)

# Unity 5 Screen Space Reflection

An open source screen space reflection for Unity 5 made by https://twitter.com/Xerxes1138

# Features

* Previous frame reprojection giving one free bounce reflection.
* Convolved scene buffer mip chain.

![Mip Chain](https://github.com/Xerxes1138/UnitySSR/blob/master/MipChain.png)


# Requirements

UnitySSR require Unity 5.4 and a shader model 3.0 ( dx9 ) graphic card.

# How to use

Set project to Linear color space and deferred shading.

Then select your main camera and go to "cCharkes/Image Effects/Rendering/Unity 5 Screen Space Reflection" or drag and drop the unitySSR.cs to your main camera inspector.

# References

- [Michal Valient, Siggraph14] "Reflections and Volumetrics of Killzone Shadow Fall"
- [Tomasz Stachowiak and Yasin Uludag, Siggraph15] "Stochastic Screen-Space Reflections"
- [Michele Giacolone, 2016] "Screen Space Reflections in The Surge"
