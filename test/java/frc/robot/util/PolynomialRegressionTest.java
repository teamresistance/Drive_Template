package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

@DisplayName("PolynomialRegression Tests")
class PolynomialRegressionTest {

  private static final double DELTA = 1e-3;

  @Test
  @DisplayName("Linear regression with perfect fit")
  void testLinearRegressionPerfectFit() {
    // y = 2x + 1
    double[] x = {0, 1, 2, 3, 4};
    double[] y = {1, 3, 5, 7, 9};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    assertEquals(1, regression.degree());
    assertEquals(1.0, regression.beta(0), DELTA); // intercept
    assertEquals(2.0, regression.beta(1), DELTA); // slope
    assertEquals(1.0, regression.r2(), DELTA); // perfect fit
  }

  @Test
  @DisplayName("Quadratic regression with perfect fit")
  void testQuadraticRegressionPerfectFit() {
    // y = x^2 + 2x + 3
    double[] x = {0, 1, 2, 3, 4};
    double[] y = {3, 6, 11, 18, 27};

    PolynomialRegression regression = new PolynomialRegression(x, y, 2);

    assertEquals(2, regression.degree());
    assertEquals(3.0, regression.beta(0), DELTA); // constant
    assertEquals(2.0, regression.beta(1), DELTA); // linear
    assertEquals(1.0, regression.beta(2), DELTA); // quadratic
    assertEquals(1.0, regression.r2(), DELTA); // perfect fit
  }

  @Test
  @DisplayName("Predict method returns correct values")
  void testPredict() {
    // y = 2x + 1
    double[] x = {0, 1, 2, 3};
    double[] y = {1, 3, 5, 7};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    assertEquals(1.0, regression.predict(0), DELTA);
    assertEquals(3.0, regression.predict(1), DELTA);
    assertEquals(5.0, regression.predict(2), DELTA);
    assertEquals(11.0, regression.predict(5), DELTA); // extrapolation
  }

  @Test
  @DisplayName("Constant function has R^2 of 1")
  void testConstantFunction() {
    double[] x = {1, 2, 3, 4, 5};
    double[] y = {5, 5, 5, 5, 5};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    assertEquals(1.0, regression.r2(), DELTA);
    assertEquals(5.0, regression.beta(0), DELTA);
  }

  @Test
  @DisplayName("Custom variable name is used in toString")
  void testCustomVariableName() {
    double[] x = {0, 1, 2};
    double[] y = {0, 1, 2};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1, "t");

    String result = regression.toString();
    assertTrue(result.contains("t"));
    assertTrue(result.contains("R^2"));
  }

  @Test
  @DisplayName("toString includes R^2 value")
  void testToStringIncludesR2() {
    double[] x = {0, 1, 2};
    double[] y = {1, 3, 5};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    String result = regression.toString();
    assertTrue(result.contains("R^2"));
  }

  @Test
  @DisplayName("CompareTo works correctly for equal polynomials")
  void testCompareToEqual() {
    double[] x = {0, 1, 2};
    double[] y = {1, 3, 5};

    PolynomialRegression regression1 = new PolynomialRegression(x, y, 1);
    PolynomialRegression regression2 = new PolynomialRegression(x, y, 1);

    assertEquals(0, regression1.compareTo(regression2));
  }

  @Test
  @DisplayName("CompareTo works correctly for different polynomials")
  void testCompareToDifferent() {
    double[] x = {0, 1, 2};
    double[] y1 = {1, 2, 3}; // y = x + 1
    double[] y2 = {2, 4, 6}; // y = 2x + 2

    PolynomialRegression regression1 = new PolynomialRegression(x, y1, 1);
    PolynomialRegression regression2 = new PolynomialRegression(x, y2, 1);

    assertTrue(regression1.compareTo(regression2) != 0);
  }

  @Test
  @DisplayName("Cubic regression")
  void testCubicRegression() {
    // y = x^3
    double[] x = {0, 1, 2, 3};
    double[] y = {0, 1, 8, 27};

    PolynomialRegression regression = new PolynomialRegression(x, y, 3);

    assertEquals(3, regression.degree());
    assertEquals(1.0, regression.r2(), DELTA); // perfect fit
    assertEquals(0.0, regression.predict(0), DELTA);
    assertEquals(8.0, regression.predict(2), DELTA);
  }

  @Test
  @DisplayName("Beta returns 0.0 for very small values")
  void testBetaReturnsZeroForSmallValues() {
    double[] x = {0, 1, 2};
    double[] y = {0, 0, 0};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    // Should return 0.0 for values close to zero
    assertEquals(0.0, regression.beta(0), DELTA);
  }

  @Test
  @DisplayName("Regression with noisy data has lower R^2")
  void testNoisyData() {
    // Approximately y = x, but with noise
    double[] x = {0, 1, 2, 3, 4};
    double[] y = {0.1, 1.2, 1.9, 3.1, 4.2};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    // R^2 should be less than 1 but still high
    assertTrue(regression.r2() < 1.0);
    assertTrue(regression.r2() > 0.9);
  }

  @Test
  @DisplayName("Degree reduction when Vandermonde matrix not full rank")
  void testDegreeReduction() {
    // Only two distinct points - can't fit degree 3
    double[] x = {1, 1};
    double[] y = {2, 2};

    PolynomialRegression regression = new PolynomialRegression(x, y, 3);

    // Degree should be reduced to fit the data
    assertTrue(regression.degree() < 3);
  }

  @Test
  @DisplayName("Negative coefficients handled correctly")
  void testNegativeCoefficients() {
    // y = -x + 5
    double[] x = {0, 1, 2, 3};
    double[] y = {5, 4, 3, 2};

    PolynomialRegression regression = new PolynomialRegression(x, y, 1);

    assertEquals(5.0, regression.beta(0), DELTA);
    assertEquals(-1.0, regression.beta(1), DELTA);
  }
}
