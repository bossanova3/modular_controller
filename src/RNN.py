import torch.nn as nn

class MovimientoDetectorRNN(nn.Module):
    def __init__(self, input_size=3, hidden_size=64, num_layers=1):
        super().__init__()
        self.rnn = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Sigmoid()

    def forward(Self, x):
        _, (hn, _) = self.rnn(x)
        out = self.fc(hn[-1])
        return self.sigmoid(out)