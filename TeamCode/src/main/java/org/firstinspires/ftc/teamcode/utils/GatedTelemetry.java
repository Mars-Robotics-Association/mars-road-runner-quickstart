package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * {@link Telemetry} decorator that can drop {@code addData}/{@code addLine}/{@code speak} (and
 * related mutators) while still forwarding configuration and optional forced flushes.
 *
 * <p>Used by {@code MarsLinearOpMode} so interactive control can run faster than the DS/Dashboard
 * publish rate: callers may write telemetry every loop; only open frames pay formatting cost, and
 * only those frames are flushed with {@link #update()} / {@link #forceUpdate()}.
 */
public final class GatedTelemetry implements Telemetry {
    private final Telemetry delegate;
    private final Log gatedLog;
    private final Item noopItem = new NoopItem();
    private final Line noopLine = new NoopLine();
    private boolean open = true;

    public GatedTelemetry(Telemetry delegate) {
        this.delegate = delegate;
        this.gatedLog = new GatedLog(delegate.log());
    }

    /**
     * When false, {@code addData}/{@code addLine}/{@code speak}/{@code clear} and {@link #update()}
     * are no-ops. Configuration setters still forward.
     */
    public void setOpen(boolean open) {
        this.open = open;
    }

    public boolean isOpen() {
        return open;
    }

    /** Always flushes the delegate (used by the frame scheduler). */
    public boolean forceUpdate() {
        return delegate.update();
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        if (!open) {
            return noopItem;
        }
        return delegate.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        if (!open) {
            return noopItem;
        }
        return delegate.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        if (!open) {
            return noopItem;
        }
        return delegate.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        if (!open) {
            return noopItem;
        }
        return delegate.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        if (!open || item instanceof NoopItem) {
            return false;
        }
        return delegate.removeItem(item);
    }

    @Override
    public void clear() {
        if (open) {
            delegate.clear();
        }
    }

    @Override
    public void clearAll() {
        if (open) {
            delegate.clearAll();
        }
    }

    @Override
    public Object addAction(Runnable action) {
        // Actions run on flush; only register while open so closed frames stay cheap.
        if (!open) {
            return action;
        }
        return delegate.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        if (!open) {
            return false;
        }
        return delegate.removeAction(token);
    }

    @Override
    public void speak(String text) {
        if (open) {
            delegate.speak(text);
        }
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        if (open) {
            delegate.speak(text, languageCode, countryCode);
        }
    }

    @Override
    public boolean update() {
        if (!open) {
            return false;
        }
        return delegate.update();
    }

    @Override
    public Line addLine() {
        if (!open) {
            return noopLine;
        }
        return delegate.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        if (!open) {
            return noopLine;
        }
        return delegate.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        if (!open || line instanceof NoopLine) {
            return false;
        }
        return delegate.removeLine(line);
    }

    @Override
    public boolean isAutoClear() {
        return delegate.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        delegate.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return delegate.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        delegate.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return delegate.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        delegate.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return delegate.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        delegate.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        delegate.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return gatedLog;
    }

    private final class GatedLog implements Log {
        private final Log delegateLog;

        GatedLog(Log delegateLog) {
            this.delegateLog = delegateLog;
        }

        @Override
        public int getCapacity() {
            return delegateLog.getCapacity();
        }

        @Override
        public void setCapacity(int capacity) {
            delegateLog.setCapacity(capacity);
        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return delegateLog.getDisplayOrder();
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {
            delegateLog.setDisplayOrder(displayOrder);
        }

        @Override
        public void add(String entry) {
            if (open) {
                delegateLog.add(entry);
            }
        }

        @Override
        public void add(String format, Object... args) {
            if (open) {
                delegateLog.add(format, args);
            }
        }

        @Override
        public void clear() {
            if (open) {
                delegateLog.clear();
            }
        }
    }

    private final class NoopItem implements Item {
        @Override
        public String getCaption() {
            return "";
        }

        @Override
        public Item setCaption(String caption) {
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            return this;
        }

        @Override
        public Item setValue(Object value) {
            return this;
        }

        @Override
        public <T> Item setValue(Func<T> valueProducer) {
            return this;
        }

        @Override
        public <T> Item setValue(String format, Func<T> valueProducer) {
            return this;
        }

        @Override
        public Item setRetained(Boolean retained) {
            return this;
        }

        @Override
        public boolean isRetained() {
            return false;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return this;
        }

        @Override
        public Item addData(String caption, Object value) {
            return this;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return this;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return this;
        }
    }

    private final class NoopLine implements Line {
        @Override
        public Item addData(String caption, String format, Object... args) {
            return noopItem;
        }

        @Override
        public Item addData(String caption, Object value) {
            return noopItem;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return noopItem;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return noopItem;
        }
    }
}
