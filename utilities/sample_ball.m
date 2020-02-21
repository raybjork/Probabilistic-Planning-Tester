% samplestate space in a ball given xy coordinates and a radius
function q = sample_ball(xc, yc, radius)
    theta = rand*(2*pi);
    r = sqrt(rand)*radius;
    q = [xc + r.*cos(theta), yc + r.*sin(theta)];
end