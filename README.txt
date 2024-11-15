# CS-E5520 Advanced Computer Graphics, Spring 2024
# Lehtinen / Härkönen, Timonen
#
# Assignment 3: Path Tracing

Student name: Huang Yaojun
Student number: 101627324

# Which parts of the assignment did you complete? Mark them 'done'.
# You can also mark non-completed parts as 'attempted' if you spent a fair amount of
# effort on them. If you do, explain the work you did in the problems/bugs section
# and leave your 'attempt' code in place (commented out if necessary) so we can see it.

R1 Integrate your raytracer (1p): done
  R2 Direct light & shadows (2p): done
  R3 Bounced indirect light (5p): done
        R4 Russian roulette (2p): done

Remember the other mandatory deliverables!
- 6 images rendered with our default presets
- Rendering competition PDF + images in separate folder

# Did you do any extra credit work?

1. glossy reflection model with Frostbite 3 standard material BRDF(diffuse + specular)
2. tangent space normal mapping
3. joint bilateral filtering for denoising final image(a toggle to enable the feature, a slider to change the filter kernel radius and ray tracing sample per pixel)

# Have you done extra credit work on previous rounds whose grading we have postponed to this round?

No

# Are there any known problems/bugs remaining in your code?

1. The joint bilateral filtering is slow since it keeps updating every frame, and creates blur effect with textures(although it gradually becomes clear as the scene converge), 
this can be solved by applying demodulate albedo technique, using another algorithm can also make the filter faster. What's more, the current implementation processes the image every frame, but it is better to do the process at final pass.

2. The normal mapping works in cornell_chesterfield but creates dark spot/noise in crytek-sponza, the cause in still being investigated.

# Did you collaborate with anyone in the class?

No.

# Any other comments you'd like to share about the assignment or the course overall?

No.

