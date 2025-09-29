namespace WebApp.Components
{
    public class RepeatingWorker : IDisposable
    {
        private readonly TimeSpan _interval;
        private readonly Func<CancellationToken, Task> _actionAsync;
        private readonly ManualResetEventSlim _pauseGate = new(initialState: true); // true = allowed to run
        private CancellationTokenSource? _cts;
        private Thread? _thread;
        private volatile bool _isRunning;
        private volatile bool _isPaused;
        private readonly object _sync = new();

        public bool IsRunning => _isRunning;
        public bool IsPaused => _isPaused;

        public RepeatingWorker(Func<CancellationToken, Task> actionAsync, TimeSpan interval)
        {
            _actionAsync = actionAsync ?? throw new ArgumentNullException(nameof(actionAsync));
            _interval = interval;
        }

        public void Start()
        {
            lock (_sync)
            {
                if (_isRunning) return;

                _cts = new CancellationTokenSource();
                _isRunning = true;
                _isPaused = false;
                _pauseGate.Set(); // ensure open

                _thread = new Thread(RunLoop)
                {
                    IsBackground = true,
                    Name = "RepeatingWorker"
                };
                _thread.Start(_cts.Token);
            }
        }

        public void Pause()
        {
            lock (_sync)
            {
                if (!_isRunning || _isPaused) return;
                _isPaused = true;
                _pauseGate.Reset(); // block loop
            }
        }

        public void Resume()
        {
            lock (_sync)
            {
                if (!_isRunning || !_isPaused) return;
                _isPaused = false;
                _pauseGate.Set(); // unblock loop
            }
        }

        public void Stop()
        {
            Thread? thread;
            CancellationTokenSource? cts;

            lock (_sync)
            {
                if (!_isRunning) return;
                _isRunning = false;

                cts = _cts;
                _cts = null;

                // ensure loop isn't stuck in pause
                _pauseGate.Set();

                thread = _thread;
                _thread = null;
            }

            try
            {
                cts?.Cancel();
            }
            catch { /* ignore */ }

            // wait for the thread to finish
            //thread?.Join();
            cts?.Dispose();
        }

        private void RunLoop(object? state)
        {
            var token = (CancellationToken)state!;
            try
            {
                while (!token.IsCancellationRequested)
                {
                    // block here if paused
                    _pauseGate.Wait(token);

                    if (token.IsCancellationRequested)
                    {
                        break;
                    }

                    // do the work
                    _actionAsync(token).GetAwaiter().GetResult();

                    // wait for interval OR cancellation
                    if (token.WaitHandle.WaitOne(_interval))
                    {
                        break; // cancellation fired
                    }
                }
            }
            catch (OperationCanceledException) { /* normal on stop */ }
            catch (ThreadAbortException) { /* legacy, unlikely */ }
            catch (Exception)
            {
                // Consider logging; keeping worker alive is optional.
                // For simplicity, exit the loop on unhandled exceptions.
            }
            finally
            {
                // normalize state
                lock (_sync)
                {
                    _isPaused = false;
                    _isRunning = false;
                }
            }
        }

        public void Dispose()
        {
            Stop();
            _pauseGate.Dispose();
        }
    }
}
