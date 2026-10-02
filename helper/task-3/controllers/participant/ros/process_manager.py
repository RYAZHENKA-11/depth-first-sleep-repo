"""Nav2 и slam_toolbox как отдельные ОС-процессы."""
import os
import signal
import subprocess
import time


class ProcessManager:
    def __init__(self, log_dir=None):
        self._procs = []
        self._args = {}
        self._log_dir = log_dir
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

    def _popen(self, args, name):
        self._args[name] = args
        log_path = os.path.join(self._log_dir, f"{name}.log") if self._log_dir else os.devnull
        logf = open(log_path, "w")
        proc = subprocess.Popen(
            args, stdout=logf, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._procs.append((name, proc, logf))
        return proc

    def launch_slam(self, launch_file, params_file):
        return self._popen(
            ["ros2", "launch", launch_file,
             f"params_file:={params_file}", "use_sim_time:=false"],
            "slam_toolbox",
        )

    def launch_nav2(self, launch_file, params_file):
        return self._popen(
            ["ros2", "launch", launch_file,
             f"params_file:={params_file}", "use_sim_time:=false"],
            "nav2",
        )

    def any_exited(self):
        return any(proc.poll() is not None for _, proc, _ in self._procs)

    def shutdown(self, term_timeout=5.0):
        for _name, proc, _logf in self._procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.time() + term_timeout
        for _name, proc, logf in self._procs:
            remaining = max(0.0, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            logf.close()
        self._procs.clear()

    def restart_exited(self):
        """Перезапустить упавшие процессы. Возвращает имена перезапущенных."""
        restarted = []
        for i, (name, proc, logf) in enumerate(list(self._procs)):
            if proc.poll() is None:
                continue
            logf.close()
            args = self._args.get(name)
            if args is None:
                continue
            self._procs.pop(i)
            self._popen(args, name)
            restarted.append(name)
        return restarted
