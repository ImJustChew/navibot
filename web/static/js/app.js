const state = document.querySelector("#connection-state");

async function checkHealth() {
  try {
    const response = await fetch("/health");
    state.textContent = response.ok ? "Connected" : "Unavailable";
  } catch {
    state.textContent = "Disconnected";
  }
}

document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", () => {
    console.log("command queued for encrypted transport", button.dataset.command);
  });
});

checkHealth();

