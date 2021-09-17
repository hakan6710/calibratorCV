
    image_width_px = 1920
    iage_height_px = 1208

    def divisorGen(n):
        factors = list(factorGenerator(n))
        nfactors = len(factors)
        f = [0] * nfactors
        while True:
            yield reduce(lambda x, y: x*y, [factors[x][0]**f[x] for x in range(nfactors)], 1)
            i = 0
            while True:
                f[i] += 1
                if f[i] <= factors[i][1]:
                    break
                f[i] = 0
                i += 1
                if i >= nfactors:
                    return



    projection_grid_shape = (19, 12)

    for 