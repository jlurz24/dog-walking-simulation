function x, y = lissajous(tmax)

  # Start by creating the lissajous curve.
  x = [];
  y = [];
 
  # Lissajous parameters
  a = sqrt(2);
  delta = pi / 2;
  A = 8;
  B = 4;
  b = 2 * a;
  TSF = 90;

  # Iterate over timestep
  for ti = [0:1:tmax * TSF]
    x(end + 1) = A * sin(a * ti / TSF + delta) + 9;
    y(end + 1) = B * sin(b * ti / TSF);
  end;
endfunction
