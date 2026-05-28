function getPlotly() {
    if (!window.Plotly) {
        throw new Error("Plotly.js is not loaded");
    }
    return window.Plotly;
}

// ---- Linked time-axis zoom ------------------------------------------------
// All plots registered here will pan/zoom together on the x (time) axis.

const linkedPlotIds = new Set();
let syncingPlots = false;
let pendingLinkedUpdate = null;
let linkedAnimationId = 0;

function linkedUpdateFromEvent(eventData) {
    if (eventData["xaxis.autorange"]) {
        return { "xaxis.autorange": true };
    }

    if (Array.isArray(eventData["xaxis.range"])) {
        return { "xaxis.range": eventData["xaxis.range"] };
    }

    if (
        eventData["xaxis.range[0]"] !== undefined
        && eventData["xaxis.range[1]"] !== undefined
    ) {
        return {
            "xaxis.range": [
                eventData["xaxis.range[0]"],
                eventData["xaxis.range[1]"],
            ],
        };
    }

    if (
        eventData["xaxis._rangeInitial[0]"] !== undefined
        && eventData["xaxis._rangeInitial[1]"] !== undefined
    ) {
        return {
            "xaxis.range": [
                eventData["xaxis._rangeInitial[0]"],
                eventData["xaxis._rangeInitial[1]"],
            ],
        };
    }

    return null;
}

function applyLinkedUpdate(Plotly, sourceId, update) {
    pendingLinkedUpdate = { Plotly, sourceId, update };
    if (linkedAnimationId) {
        return;
    }

    linkedAnimationId = requestAnimationFrame(() => {
        linkedAnimationId = 0;
        const pending = pendingLinkedUpdate;
        pendingLinkedUpdate = null;
        if (!pending || syncingPlots) {
            return;
        }

        syncingPlots = true;
        const updates = [];
        linkedPlotIds.forEach((otherId) => {
            if (otherId === pending.sourceId) {
                return;
            }
            const other = document.getElementById(otherId);
            if (other) {
                updates.push(pending.Plotly.relayout(other, pending.update));
            }
        });
        Promise.allSettled(updates).finally(() => {
            syncingPlots = false;
        });
    });
}

function installLinkedTimeZoom(Plotly, node, containerId) {
    if (node.dataset.linkedTimeZoom === "true") {
        return;
    }

    linkedPlotIds.add(containerId);
    node.dataset.linkedTimeZoom = "true";

    const onRangeChange = (eventData) => {
        if (syncingPlots) {
            return;
        }
        const update = linkedUpdateFromEvent(eventData);
        if (!update) {
            return;
        }
        applyLinkedUpdate(Plotly, containerId, update);
    };

    node.on("plotly_relayouting", onRangeChange);
    node.on("plotly_relayout", onRangeChange);
}

// ---- Common layout helpers ------------------------------------------------

function commonLayout(yTitle) {
    return {
        margin: { t: 24, r: 18, b: 48, l: 60 },
        paper_bgcolor: "rgba(0,0,0,0)",
        plot_bgcolor: "rgba(0,0,0,0)",
        legend: { orientation: "h", x: 0, y: 1.12 },
        hovermode: "closest",
        dragmode: "pan",
        xaxis: {
            title: "Time",
            type: "date",
            showgrid: true,
            zeroline: false,
        },
        yaxis: {
            title: yTitle,
            showgrid: true,
            zeroline: false,
        },
    };
}

const commonConfig = {
    responsive: true,
    displaylogo: false,
    modeBarButtonsToRemove: ["lasso2d", "select2d"],
    scrollZoom: true,
};

// ---- Public plot functions ------------------------------------------------
// timesMs: Float64Array of Unix timestamps in milliseconds.
// Plotly's date axis accepts numeric ms-since-epoch directly — no conversion
// to ISO strings needed, saving O(n) Date object + string allocations.

export function plotMotorPositions(containerId, timesMs, panVals, tiltVals) {
    const Plotly = getPlotly();
    const node = document.getElementById(containerId);
    if (!node) {
        throw new Error(`Plot container not found: ${containerId}`);
    }

    Plotly.react(
        node,
        [
            {
                x: timesMs,
                y: panVals,
                name: "pan",
                type: "scattergl",
                mode: "markers",
                marker: { color: "#3a7dd8", size: 4, opacity: 0.75 },
                hovertemplate: "%{x}<br>pan %{y:.4f}<extra></extra>",
            },
            {
                x: timesMs,
                y: tiltVals,
                name: "tilt",
                type: "scattergl",
                mode: "markers",
                marker: { color: "#d85050", size: 4, opacity: 0.75 },
                hovertemplate: "%{x}<br>tilt %{y:.4f}<extra></extra>",
            },
        ],
        commonLayout("Angle (rad)"),
        commonConfig,
    );
    installLinkedTimeZoom(Plotly, node, containerId);
}

export function plotDistance(containerId, timesMs, estDistVals, distObsVals) {
    const Plotly = getPlotly();
    const node = document.getElementById(containerId);
    if (!node) {
        throw new Error(`Plot container not found: ${containerId}`);
    }

    Plotly.react(
        node,
        [
            {
                x: timesMs,
                y: distObsVals,
                name: "observed",
                type: "scattergl",
                mode: "markers",
                marker: { color: "#a0a0a0", size: 4, opacity: 0.60 },
                hovertemplate: "%{x}<br>dist obs %{y:.2f}<extra></extra>",
            },
            {
                x: timesMs,
                y: estDistVals,
                name: "estimated",
                type: "scattergl",
                mode: "markers",
                marker: { color: "#3a9d68", size: 4, opacity: 0.80 },
                connectgaps: false,
                hovertemplate: "%{x}<br>dist est %{y:.2f}<extra></extra>",
            },
        ],
        commonLayout("Distance (m)"),
        commonConfig,
    );
    installLinkedTimeZoom(Plotly, node, containerId);
}
