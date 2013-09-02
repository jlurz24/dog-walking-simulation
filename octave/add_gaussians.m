function x, y = add_gaussians(xorig, yorig)

  # Gaussian parameters
  pNewGauss = 0.16 / 10.0;

  # Store parameters for each gaussian
  g = [];

  # Iterate over timestep
  for(i = 1:columns(xorig))
    x(i) = xorig(i);
    y(i) = yorig(i);

    if(unifrnd(0,1) <= pNewGauss)
      # Start a new gaussian
      g(end + 1, :) = [unifrnd(-pi, pi), unifrnd(pi / 2, 8 * pi), i];
    endif;
    
    # Now iterate over all the gaussians.
    for(gi = 1:rows(g))
      # Start each gaussian shifted over to capture the full width.
      gx = i - g(gi, 3) - 3 * g(gi, 2);
      gy = g(gi, 1) * e^-(gx^2 / (2 * g(gi, 2)^2));

      # Calculate a normal
      if(i > 1)
        dxy(1,1) = xorig(i) - xorig(i - 1);
        dxy(1,2) = yorig(i) - yorig(i - 1);
      else
        dxy(1,1) = 0;
        dxy(1,2) = 0;
      end;

      r(1, 1) = dxy(1,2);
      r(1, 2) = -dxy(1,1);

      # Now calculate the unit vector.
      rl = sqrt(r(1, 1)^2 + r(1, 2)^2);
      if(rl > 0)
        ru = r / rl;
      else
        ru = r;
      end;

      # Increase the length
      rn = ru * gy;

      # Finally add the components
      x(i) = x(i) + rn(1, 1);
      y(i) = y(i) + rn(1, 2);
    end;
    
    i++;
  end;
endfunction
