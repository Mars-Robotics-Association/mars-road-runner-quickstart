package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * [Telemetry] decorator that can drop `addData`/`addLine`/`speak` (and
 * related mutators) while still forwarding configuration and optional forced flushes.
 *
 * <p>Used by `MarsLinearOpMode` so interactive control can run faster than the DS/Dashboard
 * publish rate: callers may write telemetry every loop; only open frames pay formatting cost, and
 * only those frames are flushed with [update] / [forceUpdate].
 */
class GatedTelemetry(private val delegate: Telemetry) : Telemetry {
    private val gatedLog: Telemetry.Log = GatedLog(delegate.log())
    private val noopItem: Telemetry.Item = NoopItem()
    private val noopLine: Telemetry.Line = NoopLine()
    private var open = true

    /**
     * When false, `addData`/`addLine`/`speak`/`clear` and [update]
     * are no-ops. Configuration setters still forward.
     */
    fun setOpen(open: Boolean) {
        this.open = open
    }

    fun isOpen(): Boolean {
        return open
    }

    /** Always flushes the delegate (used by the frame scheduler). */
    fun forceUpdate(): Boolean {
        return delegate.update()
    }

    override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item {
        if (!open) {
            return noopItem
        }
        return delegate.addData(caption, format, *args)
    }

    override fun addData(caption: String, value: Any?): Telemetry.Item {
        if (!open) {
            return noopItem
        }
        return delegate.addData(caption, value)
    }

    override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item {
        if (!open) {
            return noopItem
        }
        return delegate.addData(caption, valueProducer)
    }

    override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item {
        if (!open) {
            return noopItem
        }
        return delegate.addData(caption, format, valueProducer)
    }

    override fun removeItem(item: Telemetry.Item): Boolean {
        if (!open || item is NoopItem) {
            return false
        }
        return delegate.removeItem(item)
    }

    override fun clear() {
        if (open) {
            delegate.clear()
        }
    }

    override fun clearAll() {
        if (open) {
            delegate.clearAll()
        }
    }

    override fun addAction(action: Runnable): Any {
        // Actions run on flush; only register while open so closed frames stay cheap.
        if (!open) {
            return action
        }
        return delegate.addAction(action)
    }

    override fun removeAction(token: Any): Boolean {
        if (!open) {
            return false
        }
        return delegate.removeAction(token)
    }

    override fun speak(text: String) {
        if (open) {
            delegate.speak(text)
        }
    }

    override fun speak(text: String, languageCode: String, countryCode: String) {
        if (open) {
            delegate.speak(text, languageCode, countryCode)
        }
    }

    override fun update(): Boolean {
        if (!open) {
            return false
        }
        return delegate.update()
    }

    override fun addLine(): Telemetry.Line {
        if (!open) {
            return noopLine
        }
        return delegate.addLine()
    }

    override fun addLine(lineCaption: String): Telemetry.Line {
        if (!open) {
            return noopLine
        }
        return delegate.addLine(lineCaption)
    }

    override fun removeLine(line: Telemetry.Line): Boolean {
        if (!open || line is NoopLine) {
            return false
        }
        return delegate.removeLine(line)
    }

    override fun isAutoClear(): Boolean {
        return delegate.isAutoClear
    }

    override fun setAutoClear(autoClear: Boolean) {
        delegate.isAutoClear = autoClear
    }

    override fun getMsTransmissionInterval(): Int {
        return delegate.msTransmissionInterval
    }

    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        delegate.msTransmissionInterval = msTransmissionInterval
    }

    override fun getItemSeparator(): String {
        return delegate.itemSeparator
    }

    override fun setItemSeparator(itemSeparator: String) {
        delegate.itemSeparator = itemSeparator
    }

    override fun getCaptionValueSeparator(): String {
        return delegate.captionValueSeparator
    }

    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        delegate.captionValueSeparator = captionValueSeparator
    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat) {
        delegate.setDisplayFormat(displayFormat)
    }

    override fun log(): Telemetry.Log {
        return gatedLog
    }

    private inner class GatedLog(private val delegateLog: Telemetry.Log) : Telemetry.Log {
        override fun getCapacity(): Int {
            return delegateLog.capacity
        }

        override fun setCapacity(capacity: Int) {
            delegateLog.capacity = capacity
        }

        override fun getDisplayOrder(): Telemetry.Log.DisplayOrder {
            return delegateLog.displayOrder
        }

        override fun setDisplayOrder(displayOrder: Telemetry.Log.DisplayOrder) {
            delegateLog.displayOrder = displayOrder
        }

        override fun add(entry: String) {
            if (open) {
                delegateLog.add(entry)
            }
        }

        override fun add(format: String, vararg args: Any?) {
            if (open) {
                delegateLog.add(format, *args)
            }
        }

        override fun clear() {
            if (open) {
                delegateLog.clear()
            }
        }
    }

    private class NoopItem : Telemetry.Item {
        override fun getCaption(): String {
            return ""
        }

        override fun setCaption(caption: String): Telemetry.Item {
            return this
        }

        override fun setValue(format: String, vararg args: Any?): Telemetry.Item {
            return this
        }

        override fun setValue(value: Any?): Telemetry.Item {
            return this
        }

        override fun <T> setValue(valueProducer: Func<T>): Telemetry.Item {
            return this
        }

        override fun <T> setValue(format: String, valueProducer: Func<T>): Telemetry.Item {
            return this
        }

        override fun setRetained(retained: Boolean?): Telemetry.Item {
            return this
        }

        override fun isRetained(): Boolean {
            return false
        }

        override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item {
            return this
        }

        override fun addData(caption: String, value: Any?): Telemetry.Item {
            return this
        }

        override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item {
            return this
        }

        override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item {
            return this
        }
    }

    private inner class NoopLine : Telemetry.Line {
        override fun addData(caption: String, format: String, vararg args: Any?): Telemetry.Item {
            return noopItem
        }

        override fun addData(caption: String, value: Any?): Telemetry.Item {
            return noopItem
        }

        override fun <T> addData(caption: String, valueProducer: Func<T>): Telemetry.Item {
            return noopItem
        }

        override fun <T> addData(caption: String, format: String, valueProducer: Func<T>): Telemetry.Item {
            return noopItem
        }
    }
}
