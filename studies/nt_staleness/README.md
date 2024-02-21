# NT Staleness

On 2/20/24 Vasili discovered that sending duplicate updates through the python publisher
to the java listener did not work.  Peter Johnson advised to use keepDuplicates on both
sender and receiver, which means changing the listener slightly, to use a "MultiSubscriber"
which is the same as the pattern listener but with PubSubOptions.  The solution is
illustrated here, with a python client and a java server.

https://www.chiefdelphi.com/t/network-tables-constant-value-staleness-reiteration/455537
