## Project Structure

For each project, there will be a main project folder containing:

1. **Documentation folder**
   - All project-related documentation, notes, diagrams, and references.

2. **Source code folder (`src`)**
   - Contains a subdirectory for each task.
   - Each task directory may include one or more files needed to complete the task.

## 2. Git Workflow

### Branching
- **`main`** → stable code only, fully reviewed
- **`feature/taskX-username`** → work on a specific task. Replace `X` with task number and `username` with your GitHub username. Example: `feature/task1-shayaan`

### Issues
- Each subtask should have a GitHub **Issue**.
- Assign yourself when you start working on it.
- Close the Issue via **Pull Request** when the task is done.

### Commits
- Commit often. Use clear messages:
```bash
git commit -m "Task2: added volume placement function"
```
## Pull Requests

- Open a PR from your `feature/` branch to `main`.
- Include a short description of the work.
- Ask for review from at least one teammate.

---

## Time Tracking

- Keep track of your hours for university credit.
- Use the format: date - hours - task - comment
- **Tip:** update every 1–2 days for accuracy.

---

## General Workflow

1. Pick a task from GitHub Issues.
2. Create a feature branch for it.
3. Write code and commit often.
4. Push your branch to GitHub.
5. Open a Pull Request when the task is ready for review.
6. Merge into `main` after approval.
7. Update time tracking sheet at each milestone or after finishing the task.


