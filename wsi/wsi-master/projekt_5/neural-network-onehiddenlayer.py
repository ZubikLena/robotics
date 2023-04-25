import numpy as np
from scipy.special import softmax


# sieć dla przykładu jedna wartwa ukryta
class NeuralNetwork():
    def __init__(self) -> None:
        self.W1 = np.random.rand(4, 3)
        self.W2 = np.random.rand(3, 4)
        self.B1 = np.zeros(3)
        self.B2 = np.zeros(4)
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
        m = X.shape[0]
        
        self.dA2 = self.H2 - Y
        
        self.dW2 = np.matmul(self.H1.T, self.dA2)
        self.dB2 = np.sum(self.dA2, axis=0).reshape(1, -1)
        self.dH1 = np.matmul(self.dA2, self.W2.T)
        self.dA1 = np.multiply(self.dH1, self.sigmoid_grad(self.H1))
        
        self.dW1 = np.matmul(X.T, self.dA1)
        self.dB1 = np.sum(self.dA1, axis=0).reshape(1, -1)
        
    def update_weights(self, learning_rate):
        # self.W3 -= learning_rate * (self.dW3)
        # self.B3 -= learning_rate * (self.dB3[0])
        self.W2 -= learning_rate * (self.dW2)
        self.B2 -= learning_rate * (self.dB2[0])
        self.W1 -= learning_rate * (self.dW1)
        self.B1 -= learning_rate * (self.dB1[0])
        
    def train(self, X, Y, learning_rate, iters):
        for i in range(iters):
            self.forward_pass(X)
            self.backward_pass(X, Y)
            self.update_weights(learning_rate)

if __name__=='__main__':
    nn = NeuralNetwork()
    
    X = np.array([
        np.array([0.7265691281976822, 8.149191599866082, -9.251739283818212, -8.616967167509726]),
        np.array([-1.9142308306298577, 2.615579510757787, -2.2968008739813243, 6.415442078462619])
    ])
    
    y = np.array([
        np.array([0,0,0,1]),
        np.array([0,1,0,0])
    ])
    
    nn.train(X, y, 0.05, 700)
    print(nn.H2)
    