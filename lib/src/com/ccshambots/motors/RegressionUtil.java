package frc.robot.ShamLib.motors;

import java.util.List;
import java.util.OptionalDouble;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class RegressionUtil {

    /**
     * Get the slope of the line of best fit for the set of x and y
     * @param x the set of x values
     * @param y the set of y values
     * @return the slope of the line of best fit
     */
    public static double getLinearM(List<Double> x, List<Double> y) {
        if(x.size() == y.size()) {
            double n = x.size();
            double sumX = x.stream().mapToDouble(Double::doubleValue).sum();
            double sumY = y.stream().mapToDouble(Double::doubleValue).sum();

            double sumXSquared = x.stream().mapToDouble((e) -> pow(e.doubleValue(), 2)).sum();

            double sumXY = 0;
            for(int i = 0; i<x.size(); i++) {
                sumXY += x.get(0) * y.get(0);
            }

            return ((n*sumXY) - (sumX*sumY)) / ((n*sumXSquared) - (sumX*sumX));
        } else {
            throw new IllegalArgumentException("x and y sets not the same size!");
        }
    }

    /**
     * Get the pearson correlation coefficient of the set of x and y
     * @param x the set of x values
     * @param y the set of y values
     * @return the pearson correlation coefficient (r)
     */
    public static double getRValue(List<Double> x, List<Double> y) {
        if(x.size() == y.size()) {
            OptionalDouble xMeanOptional = x.stream().mapToDouble(Double::doubleValue).average();
            OptionalDouble yMeanOptional = y.stream().mapToDouble(Double::doubleValue).average();

            double xMean = xMeanOptional.orElse(0);
            double yMean = yMeanOptional.orElse(0);

            //Σ(xi - x-bar)(yx - y-bar)
            double sumXYDiff = 0;

            //Σ(xi - x-bar)^2
            double xDiffSquared = 0;
            //Σ(yi-y-bar)^2
            double yDiffSquared = 0;

            for(int i = 0; i<x.size(); i++) {
                sumXYDiff += (x.get(i) - xMean) * (y.get(i) - yMean);

                xDiffSquared+= pow(x.get(i) - xMean, 2);
                yDiffSquared+= pow(y.get(i) - yMean, 2);
            }

            //Calculate the r value of the dataset
            return sumXYDiff / (sqrt(xDiffSquared) * sqrt(yDiffSquared));

        } else {
            throw new IllegalArgumentException("x and y sets are not the same size!");
        }
    }
}
