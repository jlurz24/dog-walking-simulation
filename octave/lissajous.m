function x, y = lissajous(tmax)

  # Start by creating the lissajous curve.
  x = [];
  y = [];
 
  # Lissajous parameters
  a = sqrt(2);
  delta = pi / 2;
  A = 15;
  B = 6;
  b = 2 * a;

  # Iterate over timestep
  for ti = [0:0.01:tmax]
    x(end + 1) = A * sin(a * ti + delta);
    y(end + 1) = B * sin(b * ti);
  end;
endfunction
