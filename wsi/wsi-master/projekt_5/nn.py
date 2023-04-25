import numpy as np
from scipy.special import softmax
from sklearn.model_selection import train_test_split
from sklearn import datasets
import pandas as pd
import matplotlib.pyplot as plt
from statistics import mean


class NeuralNetwork():
    def __init__(self, input_size, hidden_size, output_size) -> None:
        self.W1 = np.random.rand(input_size, hidden_size)
        self.W2 = np.random.rand(hidden_size, output_size)
        self.B1 = np.zeros(hidden_size)
        self.B2 = np.zeros(output_size)
        self.A1 = None
        self.H1 = None
        self.A2 = None
        self.H2 = None
        self.dH2 = None
        self.dA2 = None
        self.dW2 = None
        self.dB2 = None
        self.dH1 = None
        self.dA1 = None
        self.dW1 = None
        self.dB1 = None
        self.error_mean = []
        self.error_max = []

    def sigmoid(self, x):
        return np.where(x >= 0,
                        1 / (1 + np.exp(-x)),
                        np.exp(x) / (1 + np.exp(x)))

    def softmax(self, x):
        return np.exp(x)/sum(np.exp(x))

    def sigmoid_grad(self, H):
        sigmoid = self.sigmoid(H)
        return np.array(
            [sigmoid[i] * (1.0 - sigmoid[i]) for i in range(len(H))]
            )

    def forward_pass(self, X):
        self.A1 = np.matmul(X, self.W1) + self.B1
        self.H1 = self.sigmoid(self.A1)
        self.A2 = np.matmul(self.H1, self.W2) + self.B2
        self.H2 = 2*softmax(self.A2)
        return self.H2

    def backward_pass(self, X, Y):
        self.forward_pass(X)

        self.dA2 = self.H2 - Y
        self.dW2 = np.matmul(self.H1.T, self.dA2)
        self.dB2 = np.sum(self.dA2, axis=0).reshape(1, -1)
        self.dH1 = np.matmul(self.dA2, self.W2.T)
        self.dA1 = np.multiply(self.dH1, self.sigmoid_grad(self.H1))

        self.dW1 = np.matmul(X.T, self.dA1)
        self.dB1 = np.sum(self.dA1, axis=0).reshape(1, -1)
        self.calc_error_mean()
        self.calc_error_max()

    def update_weights(self, learning_rate):
        self.W2 -= learning_rate * (self.dW2)
        self.B2 -= learning_rate * (self.dB2[0])
        self.W1 -= learning_rate * (self.dW1)
        self.B1 -= learning_rate * (self.dB1[0])

    def train(self, X, Y, learning_rate, iters):
        for i in range(iters):
            self.forward_pass(X)
            self.backward_pass(X, Y)
            self.update_weights(learning_rate)

    def predict(self, X):
        result = self.forward_pass(X)
        return np.argmax(result)

    def plot_errors_mean(self):
        plt.plot(self.error_mean)
        plt.show()

    def calc_error_mean(self):
        E = []
        for e in self.dA2:
            E.append(sum(e))
        self.error_mean.append(mean(E))

    def plot_errors_max(self):
        plt.plot(self.error_max)
        plt.show()

    def calc_error_max(self):
        E = []
        for e in self.dA2:
            E.append(sum(-e))
        self.error_max.append(min(E))


if __name__ == '__main__':
    good = 0
    bad = 0

    dataset = "wine"

    if dataset == "wine":

        data = pd.read_csv('winequality-white.csv', sep=';')
        input_lsize = len(data.columns.values) - 1
        hidden_lsize = 2*input_lsize
        output_lsize = 11

        y = data['quality'].values
        X = data.iloc[:, :-1].values

        X_train, X_test, y_train_out, y_test_out = train_test_split(X, y, test_size=0.4)

        y_train = np.array([np.zeros(output_lsize) for i in y_train_out])
        y_test = np.array([np.zeros(output_lsize) for i in y_test_out])

        for y, y_out in zip(y_train, y_train_out):
            y[y_out] = 1.0

        for y, y_out in zip(y_test, y_test_out):
            y[y_out] = 1.0

        nn = NeuralNetwork(input_lsize, hidden_lsize, output_lsize)
        nn.train(X_train, y_train, 0.01, 20)

        for x, y in zip(X_test, y_test_out):
            result = nn.predict(x)
            # print(f"Predicted: {result} Real: {y}")
            if result == y:
                good += 1
            else:
                bad += 1
        print(f"Correct/Incorrect: {good}/{bad}")
        print(good/(good+bad)*100, "%")
        # nn.plot_errors_mean()
        # nn.plot_errors_max()

    elif dataset == "iris":
        iris = datasets.load_iris()

        X, y = iris.data, iris.target

        X_train, X_test, y_train_out, y_test_out = train_test_split(X, y, test_size=0.4)

        y_train = np.array([np.zeros(3) for i in y_train_out])
        y_test = np.array([np.zeros(3) for i in y_train_out])

        for y, y_out in zip(y_train, y_train_out):
            y[y_out] = 1.0

        for y, y_out in zip(y_test, y_test_out):
            y[y_out] = 1.0

        nn = NeuralNetwork(4, 8, 3)
        nn.train(X_train, y_train, 0.01, 20)

        for x, y in zip(X_test, y_test_out):
            result = nn.predict(x)
            print(f"Predicted: {result} Real: {y}")
            if result == y:
                good += 1
            else:
                bad += 1
        print(f"Correct/Incorrect: {good}/{bad}")
        # nn.plot_errors_mean()
        # nn.plot_errors_max()

    else:
        pass
